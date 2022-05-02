#![warn(missing_docs)]
//! Rust client library for ROS 2.
//!
//! For getting started, see the [README][1].
//!
//! [1]: https://github.com/ros2-rust/ros2_rust/blob/master/README.md

mod context;
mod error;
mod node;
mod qos;
mod wait;

mod rcl_bindings;

pub use context::*;
pub use error::*;
pub use node::*;
pub use qos::*;
pub use wait::*;

use rcl_bindings::rcl_context_is_valid;
use std::time::Duration;

pub use rcl_bindings::rmw_request_id_t;

/// Polls the node for new messages and executes the corresponding callbacks.
///
/// See [`WaitSet::wait`] for the meaning of the `timeout` parameter.
///
/// This may under some circumstances return
/// [`SubscriptionTakeFailed`][1], [`ClientTakeFailed`][2], [`ServiceTakeFailed`][3] when the wait
/// set spuriously wakes up.
/// This can usually be ignored.
///
/// [1]: crate::SubscriberErrorCode
/// [2]: crate::ClientErrorCode
/// [3]: crate::ServiceErrorCode
pub fn spin_once(node: &Node, timeout: Option<Duration>) -> Result<(), RclReturnCode> {
    let live_subscriptions = node.live_subscriptions();
    let live_clients = node.live_clients();
    let live_services = node.live_services();
    let ctx = Context {
        handle: node.context.clone(),
    };
    let mut wait_set = WaitSet::new(
        live_subscriptions.len(),
        0,
        0,
        live_clients.len(),
        live_services.len(),
        0,
        &ctx,
    )?;

    for live_subscription in &live_subscriptions {
        wait_set.add_subscription(live_subscription.clone())?;
    }

    for live_client in &live_clients {
        wait_set.add_client(live_client.clone())?;
    }

    for live_service in &live_services {
        wait_set.add_service(live_service.clone())?;
    }

    let ready_entities = wait_set.wait(timeout)?;

    for ready_subscription in ready_entities.subscriptions {
        ready_subscription.execute()?;
    }

    for ready_client in ready_entities.clients {
        ready_client.execute()?;
    }

    for ready_service in ready_entities.services {
        ready_service.execute()?;
    }

    Ok(())
}

/// Convenience function for calling [`rclrs::spin_once`] in a loop.
///
/// This function additionally checks that the context is still valid.
pub fn spin(node: &Node) -> Result<(), RclReturnCode> {
    // The context_is_valid functions exists only to abstract away ROS distro differences
    #[cfg(ros_distro = "foxy")]
    // SAFETY: No preconditions for this function.
    let context_is_valid = || unsafe { rcl_context_is_valid(&mut *node.context.lock()) };
    #[cfg(not(ros_distro = "foxy"))]
    // SAFETY: No preconditions for this function.
    let context_is_valid = || unsafe { rcl_context_is_valid(&*node.context.lock()) };

    while context_is_valid() {
        if let Some(error) = spin_once(node, None).err() {
            match error {
                RclReturnCode::Timeout => continue,
                error => return Err(error),
            };
        }
    }
    Ok(())
}

pub fn spin_until_future_complete<T: Unpin + Clone>(
    node: &node::Node,
    mut future: Arc<Mutex<Box<future::RclFuture<T>>>>,
) -> Result<<future::RclFuture<T> as Future>::Output, RclReturnCode> {
    let mut cx = future::create_rcl_waker_context();

    loop {
        let context_valid = unsafe { rcl_context_is_valid(&mut *node.context.lock()) };
        if context_valid {
            if let Some(error) = spin_once(node, None).err() {
                match error {
                    RclReturnCode::Timeout => continue,
                    error => return Err(error),
                };
            };
            match Future::poll(Pin::new(&mut *future.lock()), &mut cx) {
                Poll::Ready(val) => break Ok(val),
                Poll::Pending => continue,
            };
        }
    }
}
