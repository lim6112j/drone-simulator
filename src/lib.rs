pub mod app;
pub mod control;
pub mod drone;
pub mod motor;
pub mod params;
pub mod plots;
pub mod presets;
pub mod sim;
pub mod viz;
pub mod wind;

#[cfg(test)]
mod smoke {
    use crate::params::Params;

    #[test]
    fn params_module_exists_and_constructs() {
        let p = Params { placeholder: 1.0 };
        assert_eq!(p.placeholder, 1.0);
    }
}
