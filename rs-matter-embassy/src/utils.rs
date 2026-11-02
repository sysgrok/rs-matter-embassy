pub mod futures {
    use core::future::Future;

    pub trait IntoFaillble<O, E>: Future<Output = O> + Sized {
        async fn into_fallible(self) -> Result<O, E> {
            Ok(self.await)
        }
    }

    impl<T, O, E> IntoFaillble<O, E> for T where T: Future<Output = O> {}
}
