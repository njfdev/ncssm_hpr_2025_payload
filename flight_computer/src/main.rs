use nokhwa::{
    CallbackCamera, Camera,
    pixel_format::{LumaFormat, RgbFormat, YuyvFormat},
    query,
    utils::{CameraIndex, RequestedFormat},
};

fn main() {
    let cameras = query(nokhwa::utils::ApiBackend::Auto).unwrap();
    println!("Cameras: {:#?}", cameras);

    // first camera in system
    let index = CameraIndex::Index(3);
    // request the absolute highest resolution CameraFormat that can be decoded to RGB.
    let requested = RequestedFormat::new::<LumaFormat>(
        nokhwa::utils::RequestedFormatType::AbsoluteHighestFrameRate,
    );
    // make the camera
    let mut camera = CallbackCamera::new(index, requested, move |buffer| {
        let decoded = buffer.decode_image::<LumaFormat>().unwrap();
        println!("Decoded Frame of {}", decoded.len());
        decoded.save("./test.png").unwrap();
    })
    .unwrap();

    camera.open_stream().unwrap();

    loop {}
}
