# SLAMTEC RPLiDAR Modular Camera Component (C++)

This is a [Viam module](https://docs.viam.com/manage/configuration/#modules) for [SLAMTEC's RPLiDAR](https://github.com/Slamtec/rplidar_sdk) family of lidars.


## Configure your RPLidar Camera

After creating a new camera resource and adding this module to your config, several attributes can be added to specify certain configurations of your RPLidar:


```json
{
  "model": "S1",
  "serial_port": "/dev/ttyUSB0",
  "serial_baudrate": 256000,
  "use_caching": false,
  "min_range_mm": 0.0
}
```

Edit the attributes as applicable.

## Attributes

The following attributes are available:

| Name | Type | Inclusion | Description |
| ---- | ---- | --------- | ----------- |
| `serial_port` | string | Optional | The path to the serial_port used to connect to the RPLidar. This can be found using `ls /dev/` and looking for devices with values such as `ttyUSB<X>` or `tty-usbserial-<X>`. Note: `/dev/` must be prepended to the found path. Defaults to `/dev/ttyUSB0`. |
| `model` | string | Optional | The model of RPLidar [A1, A2, A3, S1, S2, S3]. If no model or serial_baudrate are given, there may be difficulty connecting due to a mismatch between the default baudrate and the necessary baudrate of the RPLidar. |
| `serial_baudrate` | int | Optional | The baudrate to be used when serially connecting to the RPLidar. Default values are provided for each model. If no model or serial_baudrate are given this will default to 256000.  |
| `use_caching` | bool | Optional | A bool that, if true, will constantly cache point clouds via a background process. Calls to get_point_cloud will return this cached value. This feature should be used if you have systems making multiple high frequency calls to this module. Defaults to false. |
| `min_range_mm` | float | Optional | A minimum range for point cloud data. Any points with a distance below this range will be ignored and not added to the returned point cloud. This value defaults to 0 but if defined, it MUST be positive. |

### Example configuration:

```
{
  "components": [
    {
      "name": "my_rplidar",
      "attributes": {
        "serial_path": "/dev/ttyUSB0",
        "model": "S1",
        "min_range_mm": 10
      },
      "namespace": "rdk",
      "type": "camera",
      "model": "jeremyrhyde:camera:rplidar"
    }
  ]
}
```

## Known Supported Hardware

Support for the following RPLiDAR hardware is provided in through this module. The table is not complete and subject to change. In order to test out the module for your specific setup, it is recommended you run the integration tests provided.

| Devices             | Mac OSX |  Linux  |
|---------------------|---------|---------|
| A1                  |    X    |    X    | 
| A2                  |    X    |    X    | 
| A3                  |    X    |    X    | 
| S1                  |    X    |    X    | 
| S2                  |    X    |    X    | 
| S3                  |    X    |    X    | 
| C1                  |         |         | 

## Building The Module

This module is available for development on linux and darwin OS systems. To build the module from scratch run:

```
make setup
make build-module
```

once this is complete, a module will be available in the local build folder (`/build/viam-rplidar`). This can be copied to your `/usr/local/bin` folder and made an executable via `make install`. Note: any other folder locations in your path can be used for this module. 
