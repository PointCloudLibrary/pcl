#include "proctor/basic_proposer.h"

namespace pcl {
  
  namespace proctor {

    typedef struct {
      std::string id;
      float votes;
    } Candidate;

    bool operator<(const Candidate &a, const Candidate &b) {
      return a.votes > b.votes; // sort descending
    }

    void
    BasicProposer::getProposed(int max_num, Detector::Entry &query, std::map<std::string, Detector::Entry> &database, std::vector<std::string> &output) {
      // get top candidates
      std::map<std::string, Detector::Entry>::iterator database_it;

      vector<Candidate> ballot;
      for ( database_it = database.begin() ; database_it != database.end(); database_it++ ) {
        std::string target_id = (*database_it).first;
        Detector::Entry target = (*database_it).second;

        Candidate* candidate = new Candidate;
        candidate->id = target_id;
        candidate->votes = getVotes(query, target);
        ballot.push_back(*candidate);
      }

      sort(ballot.begin(), ballot.end());

      output.clear();

      vector<Candidate>::iterator ballot_it;

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

    double
    BasicProposer::getVotes(Detector::Entry &query, Detector::Entry &match) {
      int max_votes = 10; // TODO Set this to what?

      double votes = 0;
      for (unsigned int pi = 0; pi < query.indices->size(); pi++) {
        vector<int> indices;
        vector<float> distances;

        clock_t start = clock();
        StopWatch s;
        int num_found = match.tree->nearestKSearch(*query.features, pi, max_votes, indices, distances);

        for (int ri = 0; ri < num_found; ri++) {
          votes += 1. / (distances[ri] + numeric_limits<float>::epsilon());
          //votes -= distances[ri];
        }
      }

      return votes;
    }

  }

}
