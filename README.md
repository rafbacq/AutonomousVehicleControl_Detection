# Autonomous Vehicle Under Adverse Conditions
## Abstract
This review evaluates the adaptive cruise control performance of a simulated autonomous
vehicle using radar measurements with Gaussian noise. The autonomous vehicle, using the
parameters of a 2024 Mazda CX-90, uses dynamical state equations that are simulated using
independently-designed Sliding Mode control laws to reach the commanded car velocity and
then slow down to maintain a safe defined distance behind a slower forward vehicle. Unique
low-pass digital filters are applied to improve the cruise control performance using radar
measurements with Gaussian noise or to maintain the desired speed/relative distance when there
is a delay since the last radar measurement. The experimental methodology applied unique
control laws and digital filters with optimized parameters to demonstrate adaptive cruise control
using nominal and worst-case sensor measurements. The simulated vehicle’s resistance against
these degraded sensor measurements shows the effectiveness of using measurement filtering
algorithms.

# YOLO Object Detection
Implemented YOLO on Canadian Adverse Driving Conditions dataset using devkit [cadc_devkit](https://github.com/mpitropov/cadc_devkit/tree/master?tab=readme-ov-file). Used Roboflow to help train/test split and augment the data. 
