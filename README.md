# Autonomous Systems 2018/2019

Nuclear energy requires operations to be performed by remote handling, due to radiation levels. Thus, accurate vehicle
mapping and localisation in this complex environments is essential to move forward on today’s clean energy pursue.
SLAM(Simultaneous Localisation And Mapping) with an EKF approach is one method of achieving this when the reliance on external sensors to the robot is not a plausible scenario. On this project, the aim is to implement and test this algorithm using artificial landmarks to help a International Thermonuclear Experimental Reactor(ITER) remote handling transport cask prototype navigate through space.

The project was developed for the Autonomous Systems course in [Instituto Superior Técnico](https://tecnico.ulisboa.pt/), Universidade de Lisboa.

## Running it

We provide a simulator as well as real datasets.
The [real datasets](https://github.com/Mrrvm/SA/tree/master/code/data) include [odometry](https://github.com/Mrrvm/SA/tree/master/code/data/ITERdata) from the ITER prototype at a 100ms rate, [images of visual markers](https://github.com/Mrrvm/SA/tree/master/code/data/CameraData/dataset_images) taken at 1s rate, respective [.txt files](https://github.com/Mrrvm/SA/tree/master/code/data/CameraData) containing the [timestamp id bearring and range] of the markers to the robot and functions to concatenate this data.

To run the algorithm, define `sim` as 0 or 1, to use real or simulated data, respectively. If it's real data, load the respective data file from the data/ directory. Get some popcorn and watch the almost reasonable results.

## Materials and Libraries

The camera used is an [uEye LE USB 3.1 Gen 1](https://en.ids-imaging.com/store/products/cameras/usb-3-1-cameras/ueye-le-usb-3-1-gen-1/show/all.html).

The [ITER](https://www.iter.org/) prototype used was handed pre-built with NXT Lego hardware. There is a [user guide](https://github.com/Mrrvm/SA/blob/master/docs/Datasheed%20and%20Guide%20v2.pdf).

Both [ueye](https://en.ids-imaging.com/download-ueye-win32.html) and [opencv](https://opencv.org/opencv-3-0.html) version 3-3-0 were used for working with the image processing under Ubuntu 14.04.

ITER data was collected under Windows OS.

[arUco library](https://opencv.org/opencv-3-0.html) was used for the visual markers. [12 different](https://github.com/Mrrvm/SA/tree/master/arucos_used) 4x4 arucos with id from 0 to 11 and 15cm of length were used. 

The camera was calibrated using [arUco boards](https://docs.opencv.org/3.1.0/da/d13/tutorial_aruco_calibration.html) also [provided](https://github.com/Mrrvm/SA/tree/master/code/data/CameraData/arucoboard) in this repo as well as with 65 images of a chessboard, producing the [calib_arucoboard.xml](https://github.com/Mrrvm/SA/blob/master/code/data/CameraData/calib_arucoboard.xml) and [calib_chess.xml](https://github.com/Mrrvm/SA/blob/master/code/data/CameraData/calib_chess.xml).


## Team
- [Diogo Caldas de Oliveira](https://github.com/caldasdeoliveira)

- [Luís Simões](https://github.com/LuisSimoes17)

- [Mariana Martins](https://github.com/Mrrvm)

- [Miguel Paulino](https://github.com/miguelfscpaulino)

## Acknowledgements

For once, we thank Microsoft Windows for being the only OS to connect to ITER.

