pub struct LowPassFilter {
    alpha: f32,
    current_estimate: f32,
}

impl LowPassFilter {
    pub fn new(alpha: f32, initial_value: f32) -> Self {
        Self { alpha, current_estimate: initial_value }
    }

    pub fn update(&mut self, raw_measurement: f32) -> f32 {
        // Blends the new reading with the old value
        self.current_estimate = (self.alpha * raw_measurement) + ((1.0 - self.alpha) * self.current_estimate);
        self.current_estimate
    }
}