/// Based on https://www.viget.com/articles/understanding-futures-in-rust-part-1/
use core::marker::PhantomData;
use parking_lot::Mutex;
use std::future::Future;
use std::pin::Pin;
use std::sync::Arc;
use std::task::Context;
use std::task::Poll;

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
