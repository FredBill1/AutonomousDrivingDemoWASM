macro_rules! wasm_getters {
    ($struct_name:ident { $($name:ident($self_ident:ident) -> $ty:ty => $body:expr;)* }) => {
        #[wasm_bindgen]
        impl $struct_name {
            $(
                #[wasm_bindgen(getter)]
                pub fn $name(&self) -> $ty {
                    let $self_ident = self;
                    $body
                }
            )*
        }
    };
}
