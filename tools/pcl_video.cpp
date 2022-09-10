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
 *   * Neither the name of the copyright holder(s) nor the names of its
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

#include <iostream>
#include <string>
#include <thread>
#include <tide/ebml_element.h>
#include <tide/file_cluster.h>
#include <tide/segment.h>
#include <tide/simple_block.h>
#include <tide/tide_impl.h>
#include <tide/tracks.h>
#include <tide/track_entry.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/common/time.h>
#include <boost/date_time/gregorian/gregorian_types.hpp> // for date
#include <boost/date_time/posix_time/posix_time.hpp> // for local_time
#include <boost/date_time/posix_time/posix_time_types.hpp> // for time_duration

using namespace std::chrono_literals;
namespace bpt = boost::posix_time;


class Recorder
{
    public:
        Recorder(std::string const& filename, std::string const& title)
            : filename_(filename), title_(title),
            stream_(filename, std::ios::in|std::ios::out|std::ios::trunc),
            count_(0)
        {
        }

        void Callback(pcl::PointCloud<pcl::PointXYZ>::ConstPtr const& cloud)
        {
            // When creating a block, the track number must be specified. Currently,
            // all blocks belong to track 1 (because this program only records one
            // track). A timecode must also be given.  It is an offset from the
            // cluster's timecode measured in the segment's timecode scale.
            bpt::ptime blk_start(bpt::microsec_clock::local_time());
            bpt::time_duration blk_offset = blk_start - cltr_start_;
            tide::BlockElement::Ptr block(new tide::SimpleBlock(1,
                        blk_offset.total_microseconds() / 10000));
            // Here the frame data itself is added to the block
            pcl::PCLPointCloud2 blob;
            pcl::toPCLPointCloud2(*cloud, blob);
            tide::Block::FramePtr frame_ptr(new tide::Block::Frame(blob.data.begin(),
                        blob.data.end()));
            block->push_back(frame_ptr);
            cluster_->push_back(block);
            // Benchmarking
            if (++count_ == 30)
            {
                double now = pcl::getTime();
                std::cerr << "Average framerate: " <<
                    static_cast<double>(count_) / (now - last_) << "Hz\n";
                count_ = 0;
                last_ = now;
            }
            // Check if the cluster has enough data in it.
            // What "enough" is depends on your own needs. Generally, a cluster
            // shouldn't be allowed to get too big in data size or too long in time, or
            // it has an adverse affect on seeking through the file. We will aim for 1
            // second of data per cluster.
            bpt::time_duration cluster_len(
                    bpt::microsec_clock::local_time() - cltr_start_);
            if (cluster_len.total_seconds() >= 1)
            {
                // Finalise the cluster
                cluster_->finalise(stream_);
                // Create a new cluster
                cltr_start_ = bpt::microsec_clock::local_time();
                bpt::time_duration cltr_offset = cltr_start_ - seg_start_;
                cluster_.reset(new tide::FileCluster(
                            cltr_offset.total_microseconds() / 10000));
                cluster_->write(stream_);
            }
        }

        int Run()
        {
            // Write the EBML PCLHeader. This specifies that the file is an EBML
            // file, and is a Tide document.
            tide::EBMLElement ebml_el;
            ebml_el.write(stream_);

            // Open a new segment in the file. This will write some initial meta-data
            // and place some padding at the start of the file for final meta-data to
            // be written after tracks, clusters, etc. have been written.
            tide::Segment segment;
            segment.write(stream_);
            // Set up the segment information so it can be used while writing tracks
            // and clusters.
            // A UID is not required, but is highly recommended.
            boost::uuids::random_generator gen;
            boost::uuids::uuid uuid = gen();
            std::vector<char> uuid_data(uuid.size());
            std::copy(uuid.cbegin(), uuid.cend(), uuid_data.begin());
            segment.info.uid(uuid_data);
            // The filename can be nice to know.
            segment.info.filename(filename_);
            // The segment's timecode scale is possibly the most important value in the
            // segment meta-data data. Without it, timely playback of frames is not
            // possible. It has a sensible default (defined in the Tide specification),
            // but here we set it to ten milliseconds for demonstrative purposes.
            segment.info.timecode_scale(10000000);
            // The segment's date should be set. It is the somewhat-awkward value of
            // the number of seconds since the start of the millennium. Boost::Date_Time
            // to the rescue!
            bpt::ptime basis(boost::gregorian::date(2001, 1, 1));
            seg_start_ = boost::posix_time::microsec_clock::local_time();
            bpt::time_duration td = seg_start_ - basis;
            segment.info.date(td.total_microseconds() * 1000);
            // Let's give the segment an inspirational title.
            segment.info.title(title_);
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
            track->codec_name("pcl::PCLPointCloud2");
            // Adding each level 1 element (only the first occurrence, in the case of
            // clusters) to the index makes opening the file later much faster.
            segment.index.insert(std::make_pair(tracks.id(),
                        segment.to_segment_offset(stream_.tellp())));
            // Now we can write the Tracks element.
            tracks.insert(track);
            tracks.write(stream_);
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
                        segment.to_segment_offset(stream_.tellp())));

            // Set up a callback to get clouds from a grabber and write them to the
            // file.
            pcl::Grabber* interface(new pcl::OpenNIGrabber());
            std::function<void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> f (
                [this] (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&) { Callback (cloud); }
            );
            interface->registerCallback(f);
            // Start the first cluster
            cltr_start_ = bpt::microsec_clock::local_time();
            bpt::time_duration cltr_offset = cltr_start_ - seg_start_;
            cluster_.reset(new tide::FileCluster(
                        cltr_offset.total_microseconds() / 10000));
            cluster_->write(stream_);
            last_ = pcl::getTime();
            interface->start();

            std::cout << "Recording frames. Press any key to stop.\n";
            getchar();

            interface->stop();
            // Close the last open cluster
            if (cluster_)
            {
                cluster_->finalise(stream_);
            }

            // Now that the data has been written, the last thing to do is to finalise
            // the segment.
            segment.finalise(stream_);
            // And finally, close the file.
            stream_.close();

            return 0;
        }

    private:
        std::string filename_;
        std::string title_;
        std::fstream stream_;
        tide::FileCluster::Ptr cluster_;
        bpt::ptime seg_start_;
        bpt::ptime cltr_start_;
        unsigned int count_;
        double last_;
};


class Player
{
    public:
        Player(std::string const& filename)
            : filename_(filename), viewer_("PCL Video Player: " + filename)
        {
            //viewer_.setBackgroundColor(0, 0, 0);
            //viewer_.initCameraParameters();
        }

        int Run()
        {
            // Open the file and check for the EBML header. This confirms that the file
            // is an EBML file, and is a Tide document.
            std::ifstream stream(filename_, std::ios::in);
            tide::ids::ReadResult id = tide::ids::read(stream);
            if (id.first != tide::ids::EBML)
            {
                std::cerr << "File does not begin with an EBML header.\n";
                return 1;
            }
            tide::EBMLElement ebml_el;
            ebml_el.read(stream);
            if (ebml_el.doc_type() != tide::TideDocType)
            {
                std::cerr << "Specified EBML file is not a Tide document.\n";
                return 1;
            }
            if (ebml_el.read_version() > tide::TideEBMLVersion)
            {
                std::cerr << "This Tide document requires read version " <<
                    ebml_el.read_version() << ".\n";
                return 1;
            }
            if (ebml_el.doc_read_version() > tide::TideVersionMajor)
            {
                std::cerr << "This Tide document requires doc read version " <<
                    ebml_el.read_version() << ".\n";
                return 1;
            }
            std::cerr << "Found EBML header\n";

            // Open the file's segment. This will read some meta-data about the segment
            // and read (or build, if necessary) an index of the level 1 elements. With
            // this index, we will be able to quickly jump to important elements such
            // as the Tracks and the first Cluster.
            id = tide::ids::read(stream);
            if (id.first != tide::ids::Segment)
            {
                std::cerr << "Segment element not found\n";
                return 1;
            }
            tide::Segment segment;
            segment.read(stream);
            // The segment's date is stored as the number of nanoseconds since the
            // start of the millennium. Boost::Date_Time is invaluable here.
            bpt::ptime basis(boost::gregorian::date(2001, 1, 1));
            bpt::time_duration sd(bpt::microseconds(segment.info.date() / 1000));
            bpt::ptime seg_start(basis + sd);

            // The segment is now open and we can start reading its child elements. To
            // begin with, we get the tracks element (their may be more than one, if
            // the document was created by merging other documents) but generally only
            // one will exist).
            // We can guarantee that there is at least one in the index because
            // otherwise the call to segment.read() would have thrown an error.
            std::streampos tracks_pos(segment.index.find(tide::ids::Tracks)->second);
            stream.seekg(segment.to_stream_offset(tracks_pos));
            // To be sure, we can check it really is a Tracks element, but this is
            // usually not necessary.
            id = tide::ids::read(stream);
            if (id.first != tide::ids::Tracks)
            {
                std::cerr << "Tracks element not at indicated position.\n";
                return 1;
            }
            // Read the tracks
            tide::Tracks tracks;
            tracks.read(stream);
            // Now we can introspect the tracks available in the file.
            if (tracks.empty())
            {
                std::cerr << "No tracks found.\n";
                return 1;
            }
            // Let's check that the file contains the codec we expect for the first
            // track.
            if (tracks[1]->codec_id() != "pointcloud2")
            {
                std::cerr << "Track #1 has wrong codec type " <<
                    tracks[1]->codec_id() << '\n';
                return 1;
            }

            bpt::ptime pb_start(bpt::microsec_clock::local_time());

            // Now we can start reading the clusters. Get an iterator to the clusters
            // in the segment.
            // In this case, we are using a file-based cluster implementation, which
            // reads blocks from the file on demand. This is usually a better
            // option tham the memory-based cluster when the size of the stored
            // data is large.
            for (tide::Segment::FileBlockIterator block(segment.blocks_begin_file(stream));
                    block != segment.blocks_end_file(stream); ++block)
            {
                bpt::time_duration blk_offset(bpt::microseconds((
                        (block.cluster()->timecode() + block->timecode()) *
                        segment.info.timecode_scale() / 1000)));
                bpt::time_duration played_time(bpt::microsec_clock::local_time() -
                        pb_start);
                // If the current playback time is ahead of this block, skip it
                if (played_time > blk_offset)
                {
                    std::cerr << "Skipping block at " << blk_offset <<
                        " because current playback time is " << played_time << '\n';
                    continue;
                }
                // Some blocks may actually contain multiple frames in a lace.
                // In this case, we are reading blocks that do not use lacing,
                // so there is only one frame per block. This is the general
                // case; lacing is typically only used when the frame size is
                // very small to reduce overhead.
                tide::BlockElement::FramePtr frame_data(*block->begin());
                // Copy the frame data into a serialised cloud structure
                pcl::PCLPointCloud2 blob;
                blob.height = 480;
                blob.width = 640;
                pcl::PCLPointField ptype;
                ptype.name = "x";
                ptype.offset = 0;
                ptype.datatype = 7;
                ptype.count = 1;
                blob.fields.push_back(ptype);
                ptype.name = "y";
                ptype.offset = 4;
                ptype.datatype = 7;
                ptype.count = 1;
                blob.fields.push_back(ptype);
                ptype.name = "z";
                ptype.offset = 8;
                ptype.datatype = 7;
                ptype.count = 1;
                blob.fields.push_back(ptype);
                blob.is_bigendian = false;
                blob.point_step = 16;
                blob.row_step = 10240;
                blob.is_dense = false;
                blob.data.assign(frame_data->begin(), frame_data->end());
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::fromPCLPointCloud2(blob, *cloud);
                // Sleep until the block's display time. The played_time is
                // updated to account for the time spent preparing the data.
                played_time = bpt::microsec_clock::local_time() - pb_start;
                bpt::time_duration sleep_time(blk_offset - played_time);
                std::cerr << "Will sleep " << sleep_time << " until displaying block\n";
                std::this_thread::sleep_for(sleep_time);
                viewer_.showCloud(cloud);
                //viewer_.removePointCloud("1");
                //viewer_.addPointCloud(cloud, "1");
                //viewer_.spinOnce();
                //if (viewer_.wasStopped())
                //{
                    //break;
                //}
            }

            return 0;
        }

    private:
        std::string filename_;
        //pcl::visualization::PCLVisualizer viewer_;
        pcl::visualization::CloudViewer viewer_;
};


int main(int argc, char** argv)
{
    std::string filename;
    if (pcl::console::parse_argument(argc, argv, "-f", filename) < 0)
    {
        std::cerr << "Usage: " << argv[0] << " -f filename [-p] [-t title]\n";
        return 1;
    }
    std::string title("PCL 3D video");
    pcl::console::parse_argument(argc, argv, "-t", title);
    if (pcl::console::find_switch(argc, argv, "-p"))
    {
        Player player(filename);
        return player.Run();
    }
    else
    {
        Recorder recorder(filename, title);
        return recorder.Run();
    }
}

