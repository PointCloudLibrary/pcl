#ifndef PROPOSER_H
#define PROPOSER_H

#include <boost/shared_ptr.hpp>
#include <vector>
#include <map>
#include "proctor/detector.h"
#include "proctor/database_entry.h"

namespace pcl
{

  namespace proctor
  {

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

        Proposer()
        {}

        void
        setDatabase(const DatabasePtr database)
        {
          database_ = database;
        }

        virtual void
        getProposed(int max_num, Entry &query, std::vector<std::string> &input, std::vector<std::string> &output) = 0;

        virtual void
        selectBestCandidates(int max_num, vector<Candidate> &ballot, std::vector<std::string> &output);

      protected:
        DatabasePtr database_;
    };

  }
}

#endif
