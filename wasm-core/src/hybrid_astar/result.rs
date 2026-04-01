use wasm_bindgen::prelude::*;

#[wasm_bindgen]
pub struct HybridAStarResult {
    flat_path: Vec<f64>,
    explored_segments: Vec<f64>,
    explored_count: usize,
    analytic_expansions: usize,
    success: bool,
}

#[wasm_bindgen]
impl HybridAStarResult {
    #[wasm_bindgen(getter)]
    pub fn flat_path(&self) -> Vec<f64> {
        self.flat_path.clone()
    }

    #[wasm_bindgen(getter)]
    pub fn explored_segments(&self) -> Vec<f64> {
        self.explored_segments.clone()
    }

    #[wasm_bindgen(getter)]
    pub fn explored_count(&self) -> usize {
        self.explored_count
    }

    #[wasm_bindgen(getter)]
    pub fn analytic_expansions(&self) -> usize {
        self.analytic_expansions
    }

    #[wasm_bindgen(getter)]
    pub fn success(&self) -> bool {
        self.success
    }
}

impl HybridAStarResult {
    pub(super) fn new(
        flat_path: Vec<f64>,
        explored_segments: Vec<f64>,
        explored_count: usize,
        analytic_expansions: usize,
        success: bool,
    ) -> Self {
        Self {
            flat_path,
            explored_segments,
            explored_count,
            analytic_expansions,
            success,
        }
    }
}
