/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Geoffrey Biggs
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  \author Geoffrey Biggs
 */

#include <boost/date_time/gregorian/gregorian_types.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <iostream>
#include <string>
#include <tide/ebml_element.h>
#include <tide/memory_cluster.h>
#include <tide/segment.h>
#include <tide/simple_block.h>
#include <tide/tide_impl.h>
#include <tide/tracks.h>
#include <tide/track_entry.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/ros/conversions.h>

namespace bpt = boost::posix_time;


// TODO: Ugly global variables. The pointers aren't even auto.
std::fstream* stream(0);
bpt::ptime start;
bpt::ptime c_start;
bpt::time_duration c_td;
tide::MemoryCluster* cluster;
bool new_cluster(true);


void record_cb(pcl::PointCloud<pcl::PointXYZ>::ConstPtr const& cloud)
{
    // Start a new cluster if necessary.
    if (new_cluster)
    {
        cluster = new tide::MemoryCluster;
        c_start = bpt::ptime(bpt::microsec_clock::local_time());
        bpt::time_duration c_td = c_start - start;
        cluster->timecode(c_td.total_microseconds() / 10000);
        cluster->write(*stream);
        new_cluster = false;
        std::cerr << "Starting new cluster at " << cluster->timecode() << '\n';
    }
    // When creating a block, the track number must be specified. Currently,
    // all blocks belong to track 1 (because this program only records one
    // track). A timecode must also be given.  It is an offset from the
    // cluster's timecode measured in the segment's timecode scale.
    bpt::ptime b_start(bpt::microsec_clock::local_time());
    bpt::time_duration b_td = b_start - c_start;
    tide::BlockElement::Ptr block(new tide::SimpleBlock(1,
                b_td.total_microseconds() / 10000));
    // Here the frame data itself is added to the block
    sensor_msgs::PointCloud2 blob;
    pcl::toROSMsg(*cloud, blob);
    tide::Block::FramePtr frame_ptr(new tide::Block::Frame(blob.data.begin(),
                blob.data.end()));
    std::cerr << "Recording block at " << block->timecode() << " with " <<
        blob.data.size() << " bytes of data\n";
    block->push_back(frame_ptr);
    cluster->push_back(block);
    // Check if the cluster has enough data in it.
    // What "enough" is depends on your own needs. Generally, a cluster
    // shouldn't be allowed to get too big in data size or too long in time, or
    // it has an adverse affect on seeking through the file. We will aim for 5
    // seconds of data per cluster.
    bpt::time_duration cluster_len(bpt::microsec_clock::local_time() - c_start);
    if (cluster_len.total_seconds() >= 5)
    {
        // Finalise the cluster
        cluster->finalise(*stream);
        delete cluster;
        // Record that a new cluster is needed
        new_cluster = true;
    }
}


int main(int argc, char** argv)
{
    if (argc != 2)
    {
        std::cerr << "Usage: " << argv[0] << " <file name>\n";
        return 1;
    }

    // Open a new file and write the EBML Header. This specifies that the file
    // is an EBML file, and is a Tide document.
    stream = new std::fstream(argv[1], std::ios::in|std::ios::out|std::ios::trunc);
    tide::EBMLElement ebml_el;
    ebml_el.write(*stream);

    // Open a new segment in the file. This will write some initial meta-data
    // and place some padding at the start of the file for final meta-data to
    // be written after tracks, clusters, etc. have been written.
    tide::Segment segment;
    segment.write(*stream);
    // Set up the segment information so it can be used while writing tracks
    // and clusters.
    // A UID is not required, but is highly recommended.
    boost::uuids::random_generator gen;
    boost::uuids::uuid uuid = gen();
    std::vector<char> uuid_data(uuid.size());
    std::copy(uuid.begin(), uuid.end(), uuid_data.begin());
    segment.info.uid(uuid_data);
    // The filename can be nice to know.
    segment.info.filename(argv[1]);
    // The segment's timecode scale is possibly the most important value in the
    // segment meta-data data. Without it, timely playback of frames is not
    // possible. It has a sensible default (defined in the Tide specification),
    // but here we set it to ten milliseconds for demonstrative purposes.
    segment.info.timecode_scale(10000000);
    // The segment's date should be set. It is the somewhat-awkward value of
    // the number of seconds since the start of the millenium. Boost::Date_Time
    // to the rescue!
    bpt::ptime basis(boost::gregorian::date(2001, 1, 1));
    start = boost::posix_time::microsec_clock::local_time();
    bpt::time_duration td = start - basis;
    segment.info.date(td.total_seconds());
    // Let's give the segment an inspirational title.
    segment.info.title("World's first true 3D video");
    // It sometimes helps to know what created a Tide file.
    segment.info.muxing_app("libtide-0.1");
    segment.info.writing_app("pcl_video");

    // Set up the tracks meta-data and write it to the file.
    tide::Tracks tracks;
    // Each track is represented in the Tracks information by a TrackEntry.
    // This specifies such things as the track number, the track's UID and the
    // codec used.
    tide::TrackEntry::Ptr track(new tide::TrackEntry(1, 1, "pointcloud2"));
    track->name("3D video");
    track->codec_name("sensor_msgs::PointCloud2");
    // Adding each level 1 element (only the first occurance, in the case of
    // clusters) to the index makes opening the file later much faster.
    segment.index.insert(std::make_pair(tracks.id(),
                segment.to_segment_offset(stream->tellp())));
    // Now we can write the Tracks element.
    tracks.insert(track);
    tracks.write(*stream);
    // The file is now ready for writing the data. The data itself is stored in
    // clusters. Each cluster contains a number of blocks, with each block
    // containing a single frame of data. Different cluster implementations are
    // (will be) available using different optimisations. Here, we use the
    // implementation that stores all its blocks in memory before writing them
    // all to the file at once. As with the segment, clusters must be opened
    // for writing before blocks are added. Once the cluster is complete, it is
    // finalised. How many blocks each cluster contains is relatively flexible:
    // the only limitation is on the range of block timecodes that can be
    // stored. Each timecode is a signed 16-bit integer, and usually blocks
    // have timecodes that are positive, limiting the range to 32767. The unit
    // of this value is the segment's timecode scale. The default timecode
    // scale therefore gives approximately 65 seconds of total range, with 32
    // seconds being usable.
    // The first cluster will appear at this point in the file, so it is
    // recorded in the segment's index for faster file reading.
    segment.index.insert(std::make_pair(tide::ids::Cluster,
                segment.to_segment_offset(stream->tellp())));

    // Set up a callback to get clouds from a grabber and write them to the
    // file.
    pcl::Grabber* interface(new pcl::OpenNIGrabber());
    boost::function<void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> f(
            boost::bind(&record_cb, _1));
    interface->registerCallback(f);
    interface->start();

    std::cout << "Recording frames. Press any key to stop.\n";
    getchar();

    interface->stop();

    // Now that the data has been written, the last thing to do is to finalise
    // the segment.
    segment.finalise(*stream);
    // And finally, close the file.
    stream->close();

    return 0;
}

