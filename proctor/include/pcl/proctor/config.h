#ifndef CONFIG_H_
#define CONFIG_H_

namespace Config
{

  /** how many models to use in training and testing */
  const int num_models = 4;

  /** how many times to test the detector */
  const int num_trials = 1 * num_models;

}

#endif //#ifndef CONFIG_H_
