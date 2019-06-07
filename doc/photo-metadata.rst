.. _photo-metadata:

Embedded Photo Metadata
=======================

Photos produced by Parrot drones embed both standard and custom metadata,
including image capture and drone flight metadata, synchronized with the
picture acquisition time, both in JPEG and DNG (raw) formats.

As in most photo files, the metadata use the Exchangeable image file format
(Exif) version 2.31 and the ISO 16684-1 Extensible Metadata Platform (XMP)
standards. Some of the tags are standard, others are comonly used by multiple
camera manufacturers and some are custom Parrot tags (only in XMP).

To read the metadata in both JPEG and DNG photo files, standard tools such as
the exiv2_ library and tools can be used.
After installing the exiv2_ package on your system, the metadata can be listed
using the following command:

.. code-block:: console

    $ exiv2 -pa file.jpg

The following tables list the available metadata for various drone models.

Any additional metadata present and not supported by the application must be
ignored. Some of these elements may not be present in a file. For example, the
location may be unknown.

EXIF metadata
-------------

Legend:

  | *ANA-AI*: Anafi Ai
  | *J*: JPEG files
  | *D*: DNG files

+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| Tag    | Key                                        | Type      | ANA-AI | Value                                                             |
+========+============================================+===========+========+===================================================================+
| *TIFF IFD*                                                                                                                                   |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0x00FE | Exif.Image.NewSubfileType                  | LONG      | D      | Type of JPEG preview image (constant: 1)                          |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0x0100 | Exif.Image.ImageWidth                      | LONG      | D      | Width of JPEG preview in pixels (constant: 640)                   |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0x0101 | Exif.Image.ImageLength                     | LONG      | D      | Height of JPEG preview in pixels (constant: 480)                  |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0x010E | Exif.Image.ImageDescription                | ASCII     | J      | Picture date and time (e.g. "Wed, 20 Oct 2021 15:46:02 +0200"),   |
|        |                                            |           |        | or a custom title set by the application                          |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0x0102 | Exif.Image.BitsPerSample                   | SHORT     | D      | Bits per sample for the JPEG preview (constant: 8)                |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0x0103 | Exif.Image.Compression                     | SHORT     | D      | Compression ID for JPEG preview (constant: 7 i.e. JPEG)           |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0x0106 | Exif.Image.PhotometricInterpretation       | SHORT     | D      | Interpretation of the JPEG preview (constant: 6 i.e. YCbCr)       |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0x010F | Exif.Image.Make                            | ASCII     | J, D   | Manufacturer (constant: "Parrot")                                 |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0x0110 | Exif.Image.Model                           | ASCII     | J, D   | Model name (e.g. "ANAFI Ai")                                      |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0x0111 | Exif.Image.StripOffsets                    | LONG      | D      | Ignored                                                           |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0x0112 | Exif.Image.Orientation                     | SHORT     | J, D   | Image Orientation (constant: 1 i.e. horizontal)                   |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0x011A | Exif.Image.XResolution                     | RATIONAL  | J      | Pixels per inches in the X direction for print (constant: 72.0)   |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0x011B | Exif.Image.YResolution                     | RATIONAL  | J      | Pixels per inches in the Y direction for print (constant: 72.0)   |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0x0128 | Exif.Image.ResolutionUnit                  | SHORT     | J      | Resolution unit (constant: 2 i.e. inches)                         |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0x0115 | Exif.Image.SamplesPerPixel                 | SHORT     | D      | Samples per pixel of the JPEG preview (constant: 3 i.e. Y,U,V)    |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0x0116 | Exif.Image.RowsPerStrip                    | LONG      | D      | Ignored                                                           |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0x0117 | Exif.Image.StripByteCounts                 | LONG      | D      | Ignored                                                           |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0x011C | Exif.Image.PlanarConfiguration             | SHORT     | D      | Ignored                                                           |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0x0131 | Exif.Image.Software                        | ASCII     | J, D   | Full drone software version (software build identifier)           |
|        |                                            |           |        | (e.g. "anafi2-classic-7.0.0")                                     |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0x0132 | Exif.Image.DateTime                        | ASCII     | J, D   | UTC date and time of the capture (e.g. "2021:10:22 11:30:09")     |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0x014A | Exif.Image.SubIFDs                         | LONG      | D      | Ignored                                                           |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0x0211 | Exif.Image.YCbCrCoefficients               | RATIONAL  | D      | Coefficients for conversion from RGB to YCbCr in the JPEG preview |
|        |                                            |           |        | (constant: 0.299 0.587 0.114)                                     |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0x0212 | Exif.Image.YCbCrSubSampling                | SHORT     | D      | YCbCr subsampling for the JPEG preview image                      |
|        |                                            |           |        | (constant: 2 1 i.e. 4:2:2)                                        |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0x0213 | Exif.Image.YCbCrPositioning                | SHORT     | J, D   | The position of chrominance components in relation to luminance   |
|        |                                            |           |        | components for pixels (constant: 1 for DNG i.e. co-sited,         |
|        |                                            |           |        | 2 for JPEG i.e. centered)                                         |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0x0214 | Exif.Image.ReferenceBlackWhite             | RATIONAL  | D      | Reference black point and white point values                      |
|        |                                            |           |        | (constant: 0.0 255.0 128.0 255.0 128.0 255.0)                     |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0x02BC | Exif.Image.XMLPacket                       | BYTE      | D      | Ignored                                                           |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0x8769 | Exif.Image.ExifTag                         | LONG      | J, D   | Offset in bytes of the Exif IFD                                   |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0x8825 | Exif.Image.GPSTag                          | LONG      | J, D   | Offset in bytes of the GPS info IFD                               |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0xC612 | Exif.Image.DNGVersion                      | BYTE      | D      | DNG version (constant: 1 4 0 0)                                   |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0xC613 | Exif.Image.DNGBackwardVersion              | BYTE      | D      | DNG backward version (constant: 1 3 0 0)                          |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0xC614 | Exif.Image.UniqueCameraModel               | ASCII     | D      | Camera model name (e.g. "Parrot ANAFI Ai")                        |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0xC621 | Exif.Image.ColorMatrix1                    | SRATIONAL | D      | Transformation matrix that converts XYZ values to reference       |
|        |                                            |           |        | camera native color space values under the first calibration      |
|        |                                            |           |        | illuminant                                                        |
|        |                                            |           |        | (constant: 0.8018 -0.2094 -0.0504 -0.2864 1.1027 0.2102           |
|        |                                            |           |        | -0.003 0.1953 0.5487)                                             |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0xC627 | Exif.Image.AnalogBalance                   | RATIONAL  | D      | White balance gains already applied to raw values                 |
|        |                                            |           |        | (constant: 1.0 1.0 1.0)                                           |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0xC628 | Exif.Image.AsShotNeutral                   | RATIONAL  | D      | White balance selected at time of capture, encoded as the         |
|        |                                            |           |        | coordinates of a perfectly neutral color in linear reference      |
|        |                                            |           |        | space values (e.g. 0.558952 1.0 0.567627)                         |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0xC62A | Exif.Image.BaselineExposure                | SRATIONAL | D      | Baseline exposure of the camera (constant: 0.0)                   |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0xC62B | Exif.Image.BaselineNoise                   | RATIONAL  | D      | Baseline camera noise level at ISO 100 (constant: 1.0)            |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0xC62C | Exif.Image.BaselineSharpness               | RATIONAL  | D      | Baseline camera sharpness (constant: 1.0)                         |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0xC62E | Exif.Image.LinearResponseLimit             | RATIONAL  | D      | Fraction of the encoding range before non-linearity occurs        |
|        |                                            |           |        | (constant: 1.0)                                                   |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0xC633 | Exif.Image.ShadowScale                     | RATIONAL  | D      | Adobe-specific data for shadows slider sensitivity                |
|        |                                            |           |        | (constant: 1.0)                                                   |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0xC65A | Exif.Image.CalibrationIlluminant1          | SHORT     | D      | Illuminant used for color matrix calibration                      |
|        |                                            |           |        | (constant: 21 i.e. D65)                                           |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0xC65D | Exif.Image.RawDataUniqueID                 | BYTE      | D      | Ignored                                                           |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0xC6F8 | Exif.Image.ProfileName                     | ASCII     | D      | The camera profile name (constant: "Embedded")                    |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0xC6FD | Exif.Image.ProfileEmbedPolicy              | LONG      | D      | The camera profile embed policy (constant: 0)                     |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| *SubImage1*                                                                                                                                  |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
|        | Exif.SubImage1.NewSubfileType              | LONG      | D      | Type of full-resolution image (constant: 0)                       |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
|        | Exif.SubImage1.ImageWidth                  | LONG      | D      | Image width in pixels (e.g. 8000)                                 |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
|        | Exif.SubImage1.ImageLength                 | LONG      | D      | Image height in pixels (e.g. 6000)                                |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
|        | Exif.SubImage1.BitsPerSample               | SHORT     | D      | Number of bits per sample (constant: 16)                          |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
|        | Exif.SubImage1.Compression                 | SHORT     | D      | Compression applied to raw values (constant: 1 i.e. uncompressed) |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
|        | Exif.SubImage1.PhotometricInterpretation   | SHORT     | D      | The photometric interpretation for raw values                     |
|        |                                            |           |        | (constant: 32803 i.e. Color Filter Array)                         |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
|        | Exif.SubImage1.StripOffsets                | LONG      | D      | Ignored                                                           |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
|        | Exif.SubImage1.SamplesPerPixel             | SHORT     | D      | Raw samples per pixel (constant: 1)                               |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
|        | Exif.SubImage1.RowsPerStrip                | LONG      | D      | Ignored                                                           |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
|        | Exif.SubImage1.StripByteCounts             | LONG      | D      | Ignored                                                           |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
|        | Exif.SubImage1.PlanarConfiguration         | SHORT     | D      | Ignored                                                           |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
|        | Exif.SubImage1.CFARepeatPatternDim         | SHORT     | D      | X and Y dimensions of the CFA pattern (constant: 2 2)             |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
|        | Exif.SubImage1.CFAPattern                  | BYTE      | D      | CFA pattern (constant: 0 1 1 2 i.e. RGGB)                         |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
|        | Exif.SubImage1.CFAPlaneColor               | BYTE      | D      | Ignored                                                           |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
|        | Exif.SubImage1.CFALayout                   | SHORT     | D      | CFA layout (constant: 1 i.e. rectangular)                         |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
|        | Exif.SubImage1.BlackLevelRepeatDim         | SHORT     | D      | X and Y dimensions of the black level data (constant: 2 2)        |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
|        | Exif.SubImage1.BlackLevel                  | RATIONAL  | D      | Zero light encoding level for the CFA pattern                     |
|        |                                            |           |        | (e.g. 4032.0 4032.0 4032.0 4032.0 for Anafi Ai)                   |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
|        | Exif.SubImage1.WhiteLevel                  | SHORT     | D      | Fully saturated encoding level (e.g. 65472 for Anafi Ai)          |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
|        | Exif.SubImage1.DefaultScale                | RATIONAL  | D      | Pixel spatial scale (constant: 1.0 1.0)                           |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
|        | Exif.SubImage1.DefaultCropOrigin           | RATIONAL  | D      | Crop origin for valid raw pixels (constant: 0.0 0.0)              |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
|        | Exif.SubImage1.DefaultCropSize             | RATIONAL  | D      | Crop size for valid raw pixels (e.g. 8000.0 6000.0)               |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
|        | Exif.SubImage1.BayerGreenSplit             | LONG      | D      | Divergence between green pixels (constant: 0 i.e. none)           |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
|        | Exif.SubImage1.AntiAliasStrength           | RATIONAL  | D      | Camera anti-aliasing quality (constant: 1.0 i.e. strong)          |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
|        | Exif.SubImage1.BestQualityScale            | RATIONAL  | D      | Best quality scale factor (constant: 1.0)                         |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
|        | Exif.SubImage1.OpcodeList2                 | UNDEF     | D      | The list of DNG opcodes to apply to linear values; this contains  |
|        |                                            |           |        | for example vignetting correction maps                            |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| *EXIF IFD*                                                                                                                                   |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0x829A | Exif.Photo.ExposureTime                    | RATIONAL  | J, D   | Exposure time in seconds (e.g. 1/480 s)                           |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0x829D | Exif.Photo.FNumber                         | RATIONAL  | J, D   | F-Number (constant: f/2.0)                                        |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0x8822 | Exif.Photo.ExposureProgram                 | SHORT     | J, D   | Program used by the camera to set the exposure (e.g. "Auto")      |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0x8827 | Exif.Photo.ISOSpeedRatings                 | SHORT     | J, D   | ISO Speed and ISO Latitude as specified in ISO 12232 (e.g. 60)    |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0x8830 | Exif.Photo.SensitivityType                 | SHORT     | J, D   | Sensitivity type (constant: 3 "ISO Speed")                        |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0x8833 | Exif.Photo.ISOSpeed                        | LONG      | J      | ISO speed value (e.g. 60)                                         |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0x9000 | Exif.Photo.ExifVersion                     | UNDEF     | J, D   | The version of the supported Exif standard                        |
|        |                                            |           |        | (constant: "0231" i.e. version 2.31)                              |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0x9003 | Exif.Photo.DateTimeOriginal                | ASCII     | J, D   | UTC date and time of the capture (e.g. "2021:10:22 11:30:09")     |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0x9004 | Exif.Photo.DateTimeDigitized               | ASCII     | J, D   | UTC date and time of the capture (e.g. "2021:10:22 11:30:09")     |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0x9101 | Exif.Photo.ComponentsConfiguration         | UNDEF     | J      | Compressed data channels (constant: "1230" i.e. YCbCr)            |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0x9201 | Exif.Photo.ShutterSpeedValue               | SRATIONAL | J, D   | Shutter speed; the unit is the APEX setting (e.g. 1/443 s)        |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0x9202 | Exif.Photo.ApertureValue                   | RATIONAL  | J, D   | Lens aperture; the unit is the APEX value (constant: f/2.0)       |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0x9204 | Exif.Photo.ExposureBiasValue               | SRATIONAL | J, D   | Exposure bias; the unit is the APEX value                         |
|        |                                            |           |        | (e.g. -11072963/33554432 EV)                                      |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0x9207 | Exif.Photo.MeteringMode                    | SHORT     | J, D   | Metering mode of the exposure program                             |
|        |                                            |           |        | (constant: 2 i.e. center weighted average)                        |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0x9208 | Exif.Photo.LightSource                     | SHORT     | J, D   | The kind of light source (constant: "Unknown")                    |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0x9209 | Exif.Photo.Flash                           | SHORT     | J      | Status of the flash when the image was shot                       |
|        |                                            |           |        | (constant: 0 i.e. no flash)                                       |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0x920A | Exif.Photo.FocalLength                     | RATIONAL  | J, D   | The actual focal length of the lens in millimeters (e.g. 5.3 mm)  |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0x9290 | Exif.Photo.SubSecTime                      | ASCII     | J      | Milliseconds of the time of the capture                           |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0x9291 | Exif.Photo.SubSecTimeOriginal              | ASCII     | J      | Milliseconds of the time of the capture                           |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0x9292 | Exif.Photo.SubSecTimeDigitized             | ASCII     | J      | Milliseconds of the time of the capture                           |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0xA000 | Exif.Photo.FlashpixVersion                 | UNDEF     | J      | Ignored (constant: "0100")                                        |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0xA001 | Exif.Photo.ColorSpace                      | SHORT     | J      | Image color space (constant: 1 i.e. sRGB)                         |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0xA002 | Exif.Photo.PixelXDimension                 | SHORT     | J      | Valid width of the meaningful compressed image (e.g. 4000)        |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0xA003 | Exif.Photo.PixelYDimension                 | SHORT     | J      | Valid height of the meaningful compressed image (e.g. 3000)       |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0xA20E | Exif.Photo.FocalPlaneXResolution           | RATIONAL  | J      | Number of pixels in the image width (X) direction per             |
|        |                                            |           |        | FocalPlaneResolutionUnit (cm) on the camera focal plane           |
|        |                                            |           |        | (e.g. 6003.2)                                                     |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0xA20F | Exif.Photo.FocalPlaneYResolution           | RATIONAL  | J      | Number of pixels in the image height (Y) direction per            |
|        |                                            |           |        | FocalPlaneResolutionUnit (cm) on the camera focal plane           |
|        |                                            |           |        | (e.g. 6003.2)                                                     |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0xA210 | Exif.Photo.FocalPlaneResolutionUnit        | SHORT     | J      | Unit for measuring FocalPlaneXResolution and                      |
|        |                                            |           |        | FocalPlaneYResolution (constant: 3 i.e. centimeters)              |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0xA300 | Exif.Photo.FileSource                      | UNDEF     | J, D   | Image file source (constant: 3 i.e. digital still camera)         |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0xA301 | Exif.Photo.SceneType                       | SHORT     | J, D   | Type of scene (constant: 1 i.e. directly photographed)            |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0xA402 | Exif.Photo.ExposureMode                    | SHORT     | J, D   | Current exposure mode (manual, auto or bracketing)                |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0xA403 | Exif.Photo.WhiteBalance                    | SHORT     | J, D   | Current white balance mode (manual or auto)                       |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0xA405 | Exif.Photo.FocalLengthIn35mmFilm           | SHORT     | J, D   | Equivalent focal length in millimeters assuming a 35mm film       |
|        |                                            |           |        | camera (e.g.: 28.0 mm)                                            |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0xA406 | Exif.Photo.SceneCaptureType                | SHORT     | J      | Type of scene that was shot (constant: 0 i.e. standard)           |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0xA408 | Exif.Photo.Contrast                        | SHORT     | J      | Contrast processing applied by the camera (constant: normal)      |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0xA409 | Exif.Photo.Saturation                      | SHORT     | J      | Saturation processing applied by the camera (constant: normal)    |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0xA40A | Exif.Photo.Sharpness                       | SHORT     | J      | Sharpness processing applied by the camera (constant: normal)     |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0xA431 | Exif.Photo.BodySerialNumber                | ASCII     | J, D   | Drone serial number (e.g. "PI040416BA8G059745")                   |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| *GPS IFD*                                                                                                                                    |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0x0000 | Exif.GPSInfo.GPSVersionID                  | BYTE[4]   | J, D   | Version of the GPS IFD (constant: '2','3','0','0')                |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0x0001 | Exif.GPSInfo.GPSLatitudeRef                | ASCII     | J, D   | Indicates whether the latitude is north or south latitude         |
|        |                                            |           |        | (constant: "N" or "S")                                            |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0x0002 | Exif.GPSInfo.GPSLatitude                   | RATIONAL  | J, D   | Latitude in units of degrees                                      |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0x0003 | Exif.GPSInfo.GPSLongitudeRef               | ASCII     | J, D   | Indicates whether the longitude is east or west longitude         |
|        |                                            |           |        | (constant: "E" or "W")                                            |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0x0004 | Exif.GPSInfo.GPSLongitude                  | RATIONAL  | J, D   | Longitude in units of degrees                                     |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0x0005 | Exif.GPSInfo.GPSAltitudeRef                | BYTE      | J, D   | Indicates the altitude reference                                  |
|        |                                            |           |        | (0 means above sea level, 1 means below sea level)                |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0x0006 | Exif.GPSInfo.GPSAltitude                   | RATIONAL  | J, D   | Altitude in units of meters                                       |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0x0008 | Exif.GPSInfo.GPSSatellites                 | ASCII     | J, D   | GPS satellites count used for measurements                        |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0x0012 | Exif.GPSInfo.GPSMapDatum                   | ASCII     | J, D   | Geodetic survey data used by the GPS receiver                     |
|        |                                            |           |        | (constant: "WGS-84")                                              |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| *Thumbnail IFD*                                                                                                                              |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0x0103 | Exif.Thumbnail.Compression                 | SHORT     | J      | Constant: 6 (compressed)                                          |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0x011a | Exif.Thumbnail.XResolution                 | RATIONAL  | J      | Constant: 72.0                                                    |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0x011b | Exif.Thumbnail.YResolution                 | RATIONAL  | J      | Constant: 72.0                                                    |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0x0128 | Exif.Thumbnail.ResolutionUnit              | SHORT     | J      | Constant: 2                                                       |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0x0201 | Exif.Thumbnail.JPEGInterchangeFormat       | LONG      | J      | Offset in bytes to the start of the thumbnail image data          |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+
| 0x0202 | Exif.Thumbnail.JPEGInterchangeFormatLength | LONG      | J      | Size in bytes of the thumbnail image data                         |
+--------+--------------------------------------------+-----------+--------+-------------------------------------------------------------------+

XMP Metadata
------------

Legend:

  | *ANA-AI*: Anafi Ai
  | *J*: JPEG files
  | *D*: DNG files

+----------------------------------------+--------+------------------------------------------------------------------+
| Key                                    | ANA-AI | Value                                                            |
+========================================+========+==================================================================+
| *XMP schema*                                                                                                       |
+----------------------------------------+--------+------------------------------------------------------------------+
| Xmp.xmp.CreatorTool                    | D      | Drone software version (e.g. "7.0.0")                            |
+----------------------------------------+--------+------------------------------------------------------------------+
| Xmp.xmp.ModifyDate                     | J, D   | UTC date and time of the capture, ISO 8601 date format           |
|                                        |        | (e.g. "2021-10-22T11:30:09.205000+02:00")                        |
+----------------------------------------+--------+------------------------------------------------------------------+
| Xmp.xmp.CreateDate                     | J, D   | UTC date and time of the capture, ISO 8601 date format           |
|                                        |        | (e.g. "2021-10-22T11:30:09.205000+02:00")                        |
+----------------------------------------+--------+------------------------------------------------------------------+
| *Dublin core schema*                                                                                               |
+----------------------------------------+--------+------------------------------------------------------------------+
| Xmp.dc.date                            | J      | UTC date and time of the capture, ISO 8601 date format           |
|                                        |        | (e.g. "2021-10-22T11:30:09.205000+02:00")                        |
+----------------------------------------+--------+------------------------------------------------------------------+
| Xmp.dc.format                          | J, D   | "image/jpeg" or "image/dng"                                      |
+----------------------------------------+--------+------------------------------------------------------------------+
| Xmp.dc.description                     | J      | Picture date and time (e.g. 'lang="x-default" Wed, 20 Oct 2021   |
|                                        |        | 15:46:02 +0200') or a custom title set by the application        |
+----------------------------------------+--------+------------------------------------------------------------------+
| *Photoshop schema*                                                                                                 |
+----------------------------------------+--------+------------------------------------------------------------------+
| Xmp.photoshop.DateCreated              | D      | UTC date and time of the capture, ISO 8601 date format           |
|                                        |        | (e.g. "2021-10-22T11:30:09.205000+02:00")                        |
+----------------------------------------+--------+------------------------------------------------------------------+
| *TIFF schema*                                                                                                      |
+----------------------------------------+--------+------------------------------------------------------------------+
| Xmp.tiff.Copyright                     | J      | Optional copyright (e.g. "Copyright (c) 2021 Parrot Drones SAS") |
+----------------------------------------+--------+------------------------------------------------------------------+
| Xmp.tiff.Make                          | J      | Manufacturer (constant: "Parrot")                                |
+----------------------------------------+--------+------------------------------------------------------------------+
| Xmp.tiff.Model                         | J      | Model name (e.g. "ANAFI Ai")                                     |
+----------------------------------------+--------+------------------------------------------------------------------+
| Xmp.tiff.Software                      | J      | Full drone software version (software build identifier)          |
|                                        |        | (e.g. "anafi2-classic-7.0.0")                                    |
+----------------------------------------+--------+------------------------------------------------------------------+
| Xmp.tiff.Orientation                   | J      | Constant: "top, left"                                            |
+----------------------------------------+--------+------------------------------------------------------------------+
| Xmp.tiff.YCbCrPositioning              | J      | Constant: "Centered"                                             |
+----------------------------------------+--------+------------------------------------------------------------------+
| *EXIF schema*                                                                                                      |
+----------------------------------------+--------+------------------------------------------------------------------+
| Xmp.exif.GPSLatitude                   | J      | Location latitude in degrees, minutes followed by a              |
|                                        |        | letter giving the reference (N=north, S=south)                   |
|                                        |        | (e.g. "48,21.1367729995753N")                                    |
+----------------------------------------+--------+------------------------------------------------------------------+
| Xmp.exif.GPSLongitude                  | J      | Location longitude in degrees, minutes followed by a             |
|                                        |        | letter giving the reference (E=east, W=west)                     |
|                                        |        | (e.g. "2,49.15540455031244E")                                    |
+----------------------------------------+--------+------------------------------------------------------------------+
| Xmp.exif.GPSAltitude                   | J      | Location altitude in meters expressed as a fraction              |
|                                        |        | (e.g. "4971569/65536")                                           |
+----------------------------------------+--------+------------------------------------------------------------------+
| Xmp.exif.GPSAltitudeRef                | J      | Location altitude reference (e.g. "Above sea level")             |
+----------------------------------------+--------+------------------------------------------------------------------+
| Xmp.exif.ExposureTime                  | J      | Exposure time in seconds expressed as a fraction                 |
|                                        |        | (e.g. "3747359/1073741824")                                      |
+----------------------------------------+--------+------------------------------------------------------------------+
| Xmp.exif.ISOSpeedRatings               | J      | ISO sensitivity (e.g. "60")                                      |
+----------------------------------------+--------+------------------------------------------------------------------+
| Xmp.exif.ExposureBiasValue             | J      | Exposure bias expressed as a fraction                            |
|                                        |        | (e.g. "-11072963/33554432 EV")                                   |
+----------------------------------------+--------+------------------------------------------------------------------+
| Xmp.exif.DateTimeOriginal              | J      | UTC date and time of the capture, ISO 8601 date format           |
|                                        |        | (e.g. "2021-10-22T11:30:09.205000+02:00")                        |
+----------------------------------------+--------+------------------------------------------------------------------+
| *Camera schema*                                                                                                    |
+----------------------------------------+--------+------------------------------------------------------------------+
| Xmp.Camera.GPSXYAccuracy               | J, D   | Accuracy (one sigma of the gaussian distribution) of the         |
|                                        |        | horizontal location of the GPS, in meters, expressed as a        |
|                                        |        | fraction (e.g. "845389/2097152")                                 |
+----------------------------------------+--------+------------------------------------------------------------------+
| Xmp.Camera.GPSZAccuracy                | J, D   | Accuracy (one sigma of the gaussian distribution) of the         |
|                                        |        | vertical location of the GPS, in meters, expressed as a          |
|                                        |        | fraction (e.g. "5117051/8388608")                                |
+----------------------------------------+--------+------------------------------------------------------------------+
| Xmp.Camera.ModelType                   | J, D   | Type of camera model, can be either "perspective" or "fisheye"   |
+----------------------------------------+--------+------------------------------------------------------------------+
| Xmp.Camera.PrincipalPoint              | J, D   | Position of the principal point in millimeters;                  |
|                                        |        | the origin is at the top/left corner, x is positive towards the  |
|                                        |        | right, y is positive towards the bottom                          |
|                                        |        | (e.g. "3.24425673,2.43319273")                                   |
+----------------------------------------+--------+------------------------------------------------------------------+
| Xmp.Camera.PerspectiveDistortion       | J      | If a perspective model is used, the distortion coefficients      |
|                                        |        | R1, R2, R3, T1, T2, as documented in                             |
|                                        |        | https://support.pix4d.com/hc/en-us/articles/202559089            |
|                                        |        | (e.g. "0.00000000,0.00000000,0.00000000,0.00000000,0.00000000")  |
+----------------------------------------+--------+------------------------------------------------------------------+
| Xmp.Camera.PerspectiveFocalLength      | J      | If a perspective model is used, the focal length of the lens     |
|                                        |        | in units of *PerspectiveFocalLengthUnits*, expressed             |
|                                        |        | as a fraction (e.g. "527/100")                                   |
+----------------------------------------+--------+------------------------------------------------------------------+
| Xmp.Camera.PerspectiveFocalLengthUnits | J      | If a perspective model is used, the units of                     |
|                                        |        | *PerspectiveFocalLength* (e.g. "mm")                             |
+----------------------------------------+--------+------------------------------------------------------------------+
| Xmp.Camera.FisheyeAffineMatrix         | J, D   | If a fisheye model is used, the affine matrix                    |
|                                        |        | C, D, E, F as documented in                                      |
|                                        |        | https://support.pix4d.com/hc/en-us/articles/202559089            |
|                                        |        | (e.g. "10858.09570312,0.00000000,0.00000000,10858.09570312")     |
+----------------------------------------+--------+------------------------------------------------------------------+
| Xmp.Camera.FisheyeAffineSymmetric      | J, D   | If a fisheye model is used, either "1" if the                    |
|                                        |        | matrix is affine symmetric, or "0" otherwise                     |
+----------------------------------------+--------+------------------------------------------------------------------+
| Xmp.Camera.FisheyePolynomial           | J, D   | If fisheye model is used, the polynomial coefficients            |
|                                        |        | p0, p1, p2, p3, p4 defined in                                    |
|                                        |        | https://support.pix4d.com/hc/en-us/articles/202559089            |
|                                        |        | (e.g. "0,1,0.15420000,-0.77260000,0.24070001")                   |
+----------------------------------------+--------+------------------------------------------------------------------+
| Xmp.Camera.Roll                        | J, D   | Roll orientation of the camera, in degrees,                      |
|                                        |        | following the convention defined in                              |
|                                        |        | https://support.pix4d.com/hc/en-us/articles/202558969            |
|                                        |        | (e.g. "-0.041258")                                               |
+----------------------------------------+--------+------------------------------------------------------------------+
| Xmp.Camera.Pitch                       | J, D   | Pitch orientation of the camera, in degrees,                     |
|                                        |        | following the convention defined in                              |
|                                        |        | https://support.pix4d.com/hc/en-us/articles/202558969            |
|                                        |        | (e.g. "38.011101")                                               |
+----------------------------------------+--------+------------------------------------------------------------------+
| Xmp.Camera.Yaw                         | J, D   | Yaw orientation of the camera, in degrees,                       |
|                                        |        | following the convention defined in                              |
|                                        |        | https://support.pix4d.com/hc/en-us/articles/202558969            |
|                                        |        | (e.g. "146.781036")                                              |
+----------------------------------------+--------+------------------------------------------------------------------+
| Xmp.Camera.AboveGroundAltitude         | J, D   | Altitude above the ground in meters expressed as a fraction      |
|                                        |        | (e.g. "11485529/262144")                                         |
+----------------------------------------+--------+------------------------------------------------------------------+
| Xmp.Camera.FlightUUID                  | J, D   | Flight unique identifier                                         |
|                                        |        | (e.g. "E424837D31F3240CFB94681E1D293DE3")                        |
+----------------------------------------+--------+------------------------------------------------------------------+
| *Parrot Drone schema*                                                                                              |
+----------------------------------------+--------+------------------------------------------------------------------+
| Xmp.drone-parrot.CameraRollDegree      | J, D   | Roll orientation of the camera, in degrees,                      |
|                                        |        | in NED (north/east/down) coordinate system                       |
|                                        |        | (e.g. "-0.052789")                                               |
+----------------------------------------+--------+------------------------------------------------------------------+
| Xmp.drone-parrot.CameraPitchDegree     | J, D   | Pitch orientation of the camera, in degrees,                     |
|                                        |        | in NED (north/east/down) coordinate system                       |
|                                        |        | (e.g. "-51.988888")                                              |
+----------------------------------------+--------+------------------------------------------------------------------+
| Xmp.drone-parrot.CameraYawDegree       | J, D   | Yaw orientation of the camera, in degrees,                       |
|                                        |        | in NED (north/east/down) coordinate system                       |
|                                        |        | (e.g. "146.848038")                                              |
+----------------------------------------+--------+------------------------------------------------------------------+
| Xmp.drone-parrot.ModelId               | J, D   | Model identifier of the drone (e.g. "091a")                      |
+----------------------------------------+--------+------------------------------------------------------------------+
| Xmp.drone-parrot.SerialNumber          | J, D   | Drone serial number (e.g. "PI040416BA8G059745")                  |
+----------------------------------------+--------+------------------------------------------------------------------+
| Xmp.drone-parrot.SoftwareVersion       | J, D   | Drone software version (e.g. "7.0.0")                            |
+----------------------------------------+--------+------------------------------------------------------------------+
| Xmp.drone-parrot.SoftwareBuildId       | J, D   | Full drone software version (software build identifier)          |
|                                        |        | (e.g. "anafi2-classic-7.0.0")                                    |
+----------------------------------------+--------+------------------------------------------------------------------+
| Xmp.drone-parrot.UtcTsAccuracy         | J, D   | Accuracy of the UTC timestamp in nanoseconds (e.g. "26")         |
+----------------------------------------+--------+------------------------------------------------------------------+
| Xmp.drone-parrot.SecureCn              | J, D   | Common name of the drone certificate used to sign the picture    |
|                                        |        | file (e.g. "12EF5.drones.parrotdrones.com")                      |
+----------------------------------------+--------+------------------------------------------------------------------+
| Xmp.drone-parrot.BootId                | J, D   | Drone boot identifier                                            |
|                                        |        | (e.g. "E424837D31F3240CFB94681E1D293DE3")                        |
+----------------------------------------+--------+------------------------------------------------------------------+
| Xmp.drone-parrot.CustomId              | J, D   | Optional application-defined custom identifier                   |
|                                        |        | (e.g. "aa185b76-659e-432b-8e41-0c20ac2c9ba7_1615363595488")      |
+----------------------------------------+--------+------------------------------------------------------------------+
| Xmp.drone-parrot.PhotoMode             | J, D   | Mode in which the picture was captured (e.g. "GPSLapse")         |
+----------------------------------------+--------+------------------------------------------------------------------+
| Xmp.drone-parrot.PanoramaType          | J, D   | If the photo is part of a panorama, the panorama type            |
|                                        |        | for which the photo was captured (e.g. "super-wide")             |
+----------------------------------------+--------+------------------------------------------------------------------+
| Xmp.drone-parrot.SequenceNumber        | J, D   | In case of grouped photos, the sequence number of the photo      |
|                                        |        | (e.g. "0")                                                       |
+----------------------------------------+--------+------------------------------------------------------------------+
| Xmp.drone-parrot.PhotoCount            | J, D   | In case of grouped photos, the total number of photos            |
|                                        |        | (e.g. "9")                                                       |
+----------------------------------------+--------+------------------------------------------------------------------+
| Xmp.drone-parrot.CaptureTsUs           | J, D   | Accurate frame capture timestamp in microseconds on the drone    |
|                                        |        | monotonic clock since an unspecified starting point              |
|                                        |        | (e.g. "733975882")                                               |
+----------------------------------------+--------+------------------------------------------------------------------+

.. _exiv2: https://exiv2.org/index.html
