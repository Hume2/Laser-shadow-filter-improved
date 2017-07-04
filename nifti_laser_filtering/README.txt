Author:   Vladimir Kubelka(kubelvla@gmail.com)

This README concerns the problem of low-range distortion. However, the main purpose of this package is to filter out points that lay on the robot body, refer to mainpage.dox or contact Martin Pecka martin.pecka@ciirc.cvut.cz

--------------------------------------------------------------------------------------

PROBLEM STATEMENT:
 Laser range measurement is affected by a systematic error at low ranges (cca. less than 0.8m). This error depends on range, intensity and possibly other factors.

MATLAB CALIBRATION:
 Scripts in the matlab_calibration folder evaluate correction function, which is a polynomial fit into the [range,intensity] -> [range_correction] function values.

HOW TO USE:
 1. Prepare an experiment where robot observes flat surface, keep the laser in the default leveld position.
 2. Use ctu_data_logger to acquire laser scans (which are saved as .csv files)
 3. In matlab, use csv_load.m function to extract data from the csv files. MAT files are stored.
 4. Use the load_range_data.m script to load the mat files containing laser scans. You have to modify the tresholds in the eval_range_... and use_range_correction_... functions according to your caliration target.
 5. The fitsurface matlab object contains parameters of the polynomial fit you want to use for correction.

NOTE: Ask Vlada if there is something missing. 



-----------------------------------------------------------------------------------------

BACKGROUND:

Francis Colas proposed first solution, which worked well for NIFTI. However, points that are too close, this solution overcompensates the error and the result is not usable for safety estimation
purposes. The description of the Francis` approach follows:

/**
     * \brief Filter to undistort close points for Sick LMS-100 laser scanner.
     *
     * \author Francis Colas
     * \author Martin Pecka
     *
     * Undistortion description by Francis Colas follows.
     *
     * The laser range is officially supposed to start at something like 40 cm (or 60? whatever)
     * but if we look at the data we can get distances as close as a few centimetres.
     * The sad thing is that those values are wrong, they are underestimated by up to several centimetres
     * (which is big). That something you could observe from an undistorted scan specifically on the
     * ground directly below the sensor or on the sides of the tracks.
     *
     * I realized that the difference between the expected distance and the measurement by the laser was
     * actually bigger for closer points (not just a constant factor nor a constant offset) but I was too lazy to
     * precisely measure a lot of distances and compare with the values. Also, I suspect that it depends on the
     * incidence angle of the beam on the surface (which is correlated to the distance for a flat surface,
     * obviously -_-), but that's not an information available to the filtering node (I wanted something fast,
     * not a nearest-neighbour search to estimate a normal ;)).
     *
     * So, in the end, I took a function to compute the offset that would have a few characteristics:
     * continuous (you don't want to introduce weird artifacts),
     * going down to zero fast for larger values (because distances far seem ok enough so I didn't want to mess with
     * them in the advertised range of the sensor),
     * increasing for smaller values,
     * and bounded (because if there's something wrong we don't want to add a point at a completely wrong place).
     *
     * I just took the Cauchy function (no exponential to compute as in a Gaussian):
     * c(d) = \frac{\alpha}{\beta + d^2}
     * where d is the measured distance by the sensor,
     * c an additive correction,
     * and alpha and beta two strictly positive parameters.
     *
     * In order to find those parameters, I just measured the distance in two points (top of side track
     * and directly below the sensor) and computed the correction by comparing to the values by the sensor.
     *
     * You can solve the system of equations and find the two formulas in the code below,
     * I thought that it'd be simpler to modify the input data for the fit rather that those magical parameters.
     *
     * Interestingly, given the values of the corrections in the code, you can see that beta is negative!
     * That means that the function is not bounded as expected. However, it's small enough that the point
     * it's wrong is inside the sensor itself (1.8 cm) but starting at 10cm returned by the sensor
     * (I believe I had checked but never got anything much below that), the correction is less than
     * 5cm (~5mm correction at 30cm; and ~1mm at 70cm).
     *
     * \image html distorted.png "Laser scan without undistorting correction"
     * \image html undistorted.png "Laser scan with undistorting correction"
     */


