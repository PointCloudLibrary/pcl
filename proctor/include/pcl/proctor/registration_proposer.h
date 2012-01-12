#ifndef REGISTRATION_PROPOSER_H
#define REGISTRATION_PROPOSER_H

#include <boost/shared_ptr.hpp>
#include <vector>
#include <map>
#include "proctor/proposer.h"

namespace pcl
{

  namespace proctor
  {

    class RegistrationProposer : public Proposer {
      public:
        typedef boost::shared_ptr<Proposer> Ptr;
        typedef boost::shared_ptr<const Proposer> ConstPtr;

        RegistrationProposer()
        {}

        virtual void
        getProposed(int max_num, Entry &query, std::vector<std::string> &input, std::vector<std::string> &output);

        double
        computeRegistration(Entry &source, Entry &target);

      private:
    };

  }
}

#endif
