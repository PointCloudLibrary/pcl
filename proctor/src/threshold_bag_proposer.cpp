#include "proctor/threshold_bag_proposer.h"

namespace pcl
{
  
  namespace proctor
  {
    void
    ThresholdBagProposer::preGetProposed(Entry &query, std::vector<std::string> &input)
    {
      //std::vector<std::string>::iterator database_it;

      //for ( database_it = input.begin() ; database_it != input.end(); database_it++ ) {
        //std::string target_id = (*database_it);
        //Entry target = (*database_)[target_id];

        //int max_votes = 1;

        //double votes = 0;

        //for (unsigned int pi = 0; pi < query.features->size(); pi++) {
          //vector<int> indices;
          //vector<float> distances;

          //int num_found = target.tree->nearestKSearch(*query.features, pi, max_votes, indices, distances);

          //for (int ri = 0; ri < num_found; ri++) {
            //[> Assuming that a distance of 0 is an error <]
            //if (distances[ri] != 0)
              //feature_distances[target_id].push_back(distances[ri]);
          //}
        //}
      //}

      //double sum = 0;
      //int num_distances = 0;
      //for (auto it = feature_distances.begin(); it != feature_distances.end(); it++)
      //{
        //std::vector<float> distances = (*it).second;
        //for (int i = 0; i < distances.size(); i++)
        //{
          //sum += distances[i];
          //num_distances++;
        //}
      //}
      //mean = sum / num_distances;

      //std_dev = 0;
      //for (auto it = feature_distances.begin(); it != feature_distances.end(); it++)
      //{
        //std::vector<float> distances = (*it).second;
        //for (int i = 0; i < distances.size(); i++)
        //{
          //std_dev += pow(distances[i] - mean, 2);
        //}
      //}
      //std_dev = pow(std_dev / num_distances, 0.5);
    }

    double
    ThresholdBagProposer::getVotes(Entry &query, Entry &match)
    {
      double max_votes = 1;
      double votes = 0;

      vector<double> feature_distances;

      for (unsigned int pi = 0; pi < query.features->size(); pi++) {
        vector<int> indices;
        vector<float> distances;

        int num_found = match.tree->nearestKSearch(*query.features, pi, max_votes, indices, distances);

        for (int ri = 0; ri < num_found; ri++) {
          /* Assuming that a distance of 0 is an error */
          if (distances[ri] != 0)
            feature_distances.push_back(distances[ri]);
        }
      }

      for (int i = 0; i < feature_distances.size(); i++)
      {
        if (feature_distances[i] < threshold_)
        {
          votes += 1;
        }
      }
      votes /= feature_distances.size();

      return votes;
    }

  }

}
