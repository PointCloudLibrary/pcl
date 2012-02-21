#include "proctor/proposer.h"

namespace pcl
{

  namespace proctor
  {

    bool
    operator<(const Candidate &a, const Candidate &b)
    {
      return a.votes > b.votes; // sort descending
    }

    void
    Proposer::selectBestCandidates(int max_num, std::vector<Candidate> &ballot, std::vector<std::string> &output)
    {
      sort(ballot.begin(), ballot.end());

      output.clear();

      std::vector<Candidate>::iterator ballot_it;

      int current_num = 0;
      for (ballot_it = ballot.begin(); ballot_it != ballot.end(); ballot_it++) {
        if (current_num < max_num)
        {
          output.push_back((*ballot_it).id);
          current_num++;
        }
        else
        {
          break;
        }
      }
    }
  }

}
