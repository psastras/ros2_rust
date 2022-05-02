/// Based on https://www.viget.com/articles/understanding-futures-in-rust-part-1/
use std::future::Future;
use std::marker::PhantomData;
use std::pin::Pin;
use std::sync::Arc;
use std::task::Context;
use std::task::Poll;
use std::task::RawWaker;
use std::task::RawWakerVTable;
use std::task::Waker;

use parking_lot::Mutex;

#[derive(Default)]
pub struct RclFuture<T> {
    value: Option<T>,
}

impl<T: Default + Clone> RclFuture<T> {
    pub fn new() -> RclFuture<T> {
        Self { value: None }
    }

    pub fn set_value(&mut self, msg: T) {
        self.value = Some(msg);
    }
}

impl<T: Clone> Future for RclFuture<T> {
    type Output = T;

    fn poll(self: Pin<&mut Self>, _ctx: &mut Context) -> Poll<Self::Output> {
        if let Some(value) = &self.value {
            Poll::Ready(value.clone())
        } else {
            Poll::Pending
        }
    }
}

#[derive(Clone)]
struct RclWaker {}

fn rclwaker_wake(_s: &RclWaker) {}

fn rclwaker_wake_by_ref(_s: &RclWaker) {}

fn rclwaker_clone(s: &RclWaker) -> RawWaker {
    let arc = unsafe { Arc::from_raw(s) };
    std::mem::forget(arc.clone());
    RawWaker::new(Arc::into_raw(arc) as *const (), &VTABLE)
}

const VTABLE: RawWakerVTable = unsafe {
    RawWakerVTable::new(
        |s| rclwaker_clone(&*(s as *const RclWaker)),
        |s| rclwaker_wake(&*(s as *const RclWaker)),
        |s| rclwaker_wake_by_ref(&*(s as *const RclWaker)),
        |s| drop(Arc::from_raw(s as *const RclWaker)),
    )
};

fn rclwaker_into_waker(s: *const RclWaker) -> Waker {
    let raw_waker = RawWaker::new(s as *const (), &VTABLE);
    unsafe { Waker::from_raw(raw_waker) }
}

fn create_rcl_waker_context() {
    let rclwaker = Arc::new(RclWaker {});
    let waker = rclwaker_into_waker(Arc::into_raw(rclwaker));
    std::task::Context::from_waker(&waker)
}
