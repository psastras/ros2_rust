use crate::error::{RclReturnCode, ToResult};
use crate::qos::QoSProfile;
use crate::rcl_bindings::*;
use crate::Context;

mod client;
mod publisher;
mod service;
mod subscription;
pub use self::client::*;
pub use self::publisher::*;
pub use self::service::*;
pub use self::subscription::*;

use std::ffi::{CStr, CString};
use std::os::raw::c_char;
use std::sync::{Arc, Weak};
use std::vec::Vec;

use parking_lot::Mutex;

use rosidl_runtime_rs::Message;

impl Drop for rcl_node_t {
    fn drop(&mut self) {
        // SAFETY: No preconditions for this function
        unsafe { rcl_node_fini(self).unwrap() };
    }
}

/// A processing unit that can communicate with other nodes.
///
/// Nodes are a core concept in ROS 2. Refer to the official ["Understanding ROS 2 nodes"][1]
/// tutorial for an introduction.
///
/// Ownership of the node is shared with all [`Publisher`]s and [`Subscription`]s created from it.
/// That means that even after the node itself is dropped, it will continue to exist and be
/// displayed by e.g. `ros2 topic` as long as its publishers and subscriptions are not dropped.
///
/// [1]: https://docs.ros.org/en/rolling/Tutorials/Understanding-ROS2-Nodes.html
pub struct Node {
    handle: Arc<Mutex<rcl_node_t>>,
    pub(crate) context: Arc<Mutex<rcl_context_t>>,
    pub(crate) clients: Vec<Weak<dyn ClientBase>>,
    pub(crate) services: Vec<Weak<dyn ServiceBase>>,
    pub(crate) subscriptions: Vec<Weak<dyn SubscriptionBase>>,
}

impl Node {
    /// Creates a new node in the empty namespace.
    #[allow(clippy::new_ret_no_self)]
    pub fn new(node_name: &str, context: &Context) -> Result<Node, RclReturnCode> {
        Self::new_with_namespace("", node_name, context)
    }

    /// Creates a new node in a namespace.
    ///
    /// A namespace without a leading forward slash is automatically changed to have a leading
    /// forward slash.
    pub fn new_with_namespace(
        node_ns: &str,
        node_name: &str,
        context: &Context,
    ) -> Result<Node, RclReturnCode> {
        let raw_node_name = CString::new(node_name).unwrap();
        let raw_node_ns = CString::new(node_ns).unwrap();

        // SAFETY: Getting a zero-initialized value is always safe.
        let mut node_handle = unsafe { rcl_get_zero_initialized_node() };
        let context_handle = &mut *context.handle.lock();

        unsafe {
            // SAFETY: No preconditions for this function.
            let node_options = rcl_node_get_default_options();
            // SAFETY: The node handle is zero-initialized as expected by this function.
            // The strings and node options are copied by this function, so we don't need
            // to keep them alive.
            // The context handle is kept alive because it is co-owned by the node.
            rcl_node_init(
                &mut node_handle,
                raw_node_name.as_ptr(),
                raw_node_ns.as_ptr(),
                context_handle,
                &node_options,
            )
            .ok()?;
        }

        let handle = Arc::new(Mutex::new(node_handle));

        Ok(Node {
            handle,
            context: context.handle.clone(),
            subscriptions: std::vec![],
        })
    }

    /// Returns the name of the node.
    ///
    /// This returns the name after remapping, so it is not necessarily the same as the name that
    /// was used when creating the node.
    ///
    /// # Example
    /// ```
    /// # use rclrs::{Context, RclReturnCode};
    /// // Without remapping
    /// let context = Context::new([])?;
    /// let node = context.create_node("my_node")?;
    /// assert_eq!(node.name(), String::from("my_node"));
    /// // With remapping
    /// let remapping = ["--ros-args", "-r", "__node:=your_node"].map(String::from);
    /// let context_r = Context::new(remapping)?;
    /// let node_r = context_r.create_node("my_node")?;
    /// assert_eq!(node_r.name(), String::from("your_node"));
    /// # Ok::<(), RclReturnCode>(())
    /// ```
    pub fn name(&self) -> String {
        self.get_string(rcl_node_get_name)
    }

    /// Returns the namespace of the node.
    ///
    /// This returns the namespace after remapping, so it is not necessarily the same as the
    /// namespace that was used when creating the node.
    ///
    /// # Example
    /// ```
    /// # use rclrs::{Context, RclReturnCode};
    /// // Without remapping
    /// let context = Context::new([])?;
    /// let node = context.create_node_with_namespace("/my/namespace", "my_node")?;
    /// assert_eq!(node.namespace(), String::from("/my/namespace"));
    /// // With remapping
    /// let remapping = ["--ros-args", "-r", "__ns:=/your_namespace"].map(String::from);
    /// let context_r = Context::new(remapping)?;
    /// let node_r = context_r.create_node("my_node")?;
    /// assert_eq!(node_r.namespace(), String::from("/your_namespace"));
    /// # Ok::<(), RclReturnCode>(())
    /// ```
    pub fn namespace(&self) -> String {
        self.get_string(rcl_node_get_namespace)
    }

    /// Returns the fully qualified name of the node.
    ///
    /// The fully qualified name of the node is the node namespace combined with the node name.
    /// It is subject to the remappings shown in [`Node::name`] and [`Node::namespace`].
    ///
    /// # Example
    /// ```
    /// # use rclrs::{Context, RclReturnCode};
    /// let context = Context::new([])?;
    /// let node = context.create_node_with_namespace("/my/namespace", "my_node")?;
    /// assert_eq!(node.fully_qualified_name(), String::from("/my/namespace/my_node"));
    /// # Ok::<(), RclReturnCode>(())
    /// ```
    pub fn fully_qualified_name(&self) -> String {
        self.get_string(rcl_node_get_fully_qualified_name)
    }

    fn get_string(
        &self,
        getter: unsafe extern "C" fn(*const rcl_node_t) -> *const c_char,
    ) -> String {
        let char_ptr = unsafe {
            // SAFETY: The node handle is valid.
            getter(&*self.handle.lock())
        };
        debug_assert!(!char_ptr.is_null());
        let cstr = unsafe {
            // SAFETY: The returned CStr is immediately converted to an owned string,
            // so the lifetime is no issue. The ptr is valid as per the documentation
            // of rcl_node_get_name.
            CStr::from_ptr(char_ptr)
        };
        cstr.to_string_lossy().into_owned()
    }

    /// Creates a [`Client`][1].
    ///
    /// [1]: crate::Client
    // TODO: make client's lifetime depend on node's lifetime
    pub fn create_client<T>(
        &mut self,
        topic: &str,
    ) -> Result<Arc<crate::node::client::Client<T>>, RclReturnCode>
    where
        T: rosidl_runtime_rs::Service + 'static,
    {
        let client = Arc::new(crate::node::client::Client::<T>::new(self, topic)?);
        self.clients
            .push(Arc::downgrade(&client) as Weak<dyn ClientBase>);
        Ok(client)
    }

    /// Creates a [`Publisher`][1].
    ///
    /// [1]: crate::Publisher
    // TODO: make publisher's lifetime depend on node's lifetime
    pub fn create_publisher<T>(
        &self,
        topic: &str,
        qos: QoSProfile,
    ) -> Result<Publisher<T>, RclReturnCode>
    where
        T: Message,
    {
        Publisher::<T>::new(self, topic, qos)
    }

    /// Creates a [`Service`][1].
    ///
    /// [1]: crate::Service
    // TODO: make service's lifetime depend on node's lifetime
    pub fn create_service<T, F>(
        &mut self,
        topic: &str,
        callback: F,
    ) -> Result<Arc<crate::node::service::Service<T>>, RclReturnCode>
    where
        T: rosidl_runtime_rs::Service + 'static,
        F: FnMut(&rmw_request_id_t, &T::Request, &mut T::Response) + Sized + 'static,
    {
        let service = Arc::new(crate::node::service::Service::<T>::new(
            self, topic, callback,
        )?);
        self.services
            .push(Arc::downgrade(&service) as Weak<dyn ServiceBase>);
        Ok(service)
    }

    /// Creates a [`Subscription`][1].
    ///
    /// [1]: crate::Subscription
    // TODO: make subscription's lifetime depend on node's lifetime
    pub fn create_subscription<T, F>(
        &mut self,
        topic: &str,
        qos: QoSProfile,
        callback: F,
    ) -> Result<Arc<Subscription<T>>, RclReturnCode>
    where
        T: Message,
        F: FnMut(T) + Sized + 'static,
    {
        let subscription = Arc::new(Subscription::<T>::new(self, topic, qos, callback)?);
        self.subscriptions
            .push(Arc::downgrade(&subscription) as Weak<dyn SubscriptionBase>);
        Ok(subscription)
    }

    /// Returns the subscriptions that have not been dropped yet.
    pub(crate) fn live_subscriptions(&self) -> Vec<Arc<dyn SubscriptionBase>> {
        self.subscriptions
            .iter()
            .filter_map(Weak::upgrade)
            .collect()
    }

    pub(crate) fn live_clients(&self) -> Vec<Arc<dyn ClientBase>> {
        self.clients.iter().filter_map(Weak::upgrade).collect()
    }

    pub(crate) fn live_services(&self) -> Vec<Arc<dyn ServiceBase>> {
        self.services.iter().filter_map(Weak::upgrade).collect()
    }
}
