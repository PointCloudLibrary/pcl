#ifndef PROPOSER_H
#define PROPOSER_H

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/export.hpp>

#include <boost/shared_ptr.hpp>
#include <vector>
#include <map>
#include "proctor/database_entry.h"


namespace pcl
{

  namespace proctor
  {

    class Detector;

    struct Candidate {
      std::string id;
      double votes;
    };

    class Proposer {
      public:
        typedef boost::shared_ptr<Proposer> Ptr;
        typedef boost::shared_ptr<const Proposer> ConstPtr;

        typedef boost::shared_ptr<std::map<std::string, Entry> > DatabasePtr;
        typedef boost::shared_ptr<const std::map<std::string, Entry> > ConstDatabasePtr;

        Proposer(Detector *detector = NULL) : detector_(detector)
        {}

        void
        setDatabase(const DatabasePtr database)
        {
          database_ = database;
        }

        virtual void
        getProposed(int max_num, Entry &query, std::vector<std::string> &input, std::vector<std::string> &output) = 0;

        virtual void
        selectBestCandidates(int max_num, std::vector<Candidate> &ballot, std::vector<std::string> &output);

      protected:
        DatabasePtr database_;

        Detector *detector_;
      private:
        friend class boost::serialization::access;

        template<class Archive>
        void serialize(Archive & ar, const unsigned int version)
        {
        }
    };

  }
}

//BOOST_CLASS_EXPORT(pcl::proctor::Proposer)

#endif
