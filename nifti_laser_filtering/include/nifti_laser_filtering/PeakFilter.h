#ifndef PEAKFILTER_H
#define PEAKFILTER_H

#include <filters/filter_base.h>
#include <sensor_msgs/LaserScan.h>

namespace nifti_laser_filtering {

/**
     */
class PeakFilter : public filters::FilterBase<sensor_msgs::LaserScan> {
  public:
    virtual ~PeakFilter();

    //! Read config parameters loaded by FilterBase::configure(string, NodeHandle)
    bool configure();

    //! Apply the filter.
    bool update(const sensor_msgs::LaserScan& input_scan, sensor_msgs::LaserScan& filtered_scan);

  protected:
    //! The cosine of maximum angle of peak that is going to be filtered.
    double cos_min;

    //! The count how many times should this filter repeat.
    int passes;

    //! Makes one filter pass.
    void make_pass(sensor_msgs::LaserScan& filtered_scan, int& num_filtered_points);
};

}

#endif // PEAKFILTER_H

