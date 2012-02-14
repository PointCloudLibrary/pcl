#include <cstdio>

#include <vtkCellArray.h>
#include <vtkFloatArray.h>
#include <vtkPoints.h>

#include "proctor/proctor.h"
#include "proctor/detector.h"
#include "proctor/scanning_model_source.h"
#include "proctor/confusion_matrix.h"

#ifdef _MSC_VER
# define snprintf _snprintf
#endif

namespace pcl
{

  namespace proctor
  {

    IndicesPtr
    Proctor::randomSubset(int n, int r)
    {
      IndicesPtr subset (new vector<int>());
      vector<int> bag (n);
      for (int i = 0; i < n; i++) bag[i] = i;
      int edge = n;
      subset->resize(r);
      for (int i = 0; i < r; i++) {
        int pick = rand() % edge;
        (*subset)[i] = bag[pick];
        bag[pick] = bag[--edge];
      }
      return subset;
    }

    void
    Proctor::train(Detector &detector)
    {
      source_->getModelIDs(model_ids_);

      cout << "Proctor beginning training" << endl;

      cout << "[models]" << endl;

      //for (int mi = 0; mi < num_model_ids && mi < Config::num_models; mi++) {
      timer.start();
      const int num_model_ids = model_ids_.size();
#pragma omp parallel for
      for (int mi = 0; mi < Config::num_models; mi++) {
        std::string model_id = model_ids_[mi];
        cout << "Begin scanning model " << mi << " (" << model_id << ")" << endl;
        Scene *scene = new Scene(model_id, source_->getTrainingModel(model_id));
        cout << "Finished scanning model " << mi << " (" << model_id << ")" << endl;
        cout << endl;

        cout << "Begin training model " << mi << " (" << model_id << ")" << endl;
        // TODO Time the training phase
        detector.train(*scene);
        cout << "Finished training model " << mi << " (" << model_id << ")" << endl;
        cout << endl;
      }
      timer.stop(OBTAIN_CLOUD_TRAINING);

      cout << "Proctor finished training" << endl;
    }

    double
    Proctor::test(Detector &detector, unsigned int seed)
    {
      srand(seed);

      source_->resetTestGenerator();

      // run the tests
      memset(confusion, 0, sizeof(confusion));

      std::map<std::string, std::map<std::string, int> > guesses;
      ConfusionMatrix confusion_matrix;

      int num_model_ids = model_ids_.size();
//#pragma omp parallel for
      for (int ni = 0; num_model_ids > 0 && ni < Config::num_trials; ni++) {
        cout << "[test " << ni << "]" << endl;

        std::string truth_id = model_ids_[ni % model_ids_.size()];

        std::map<std::string, int>& guesses_for_id = guesses[truth_id];

        timer.start();
        PointCloud<PointNormal>::Ptr test_cloud = source_->getTestModel(truth_id);
        timer.stop(OBTAIN_CLOUD_TESTING);

        cout << "scanned model " << truth_id << endl;

        timer.start();
        try {
          Scene test_scene(truth_id, test_cloud);
          std::string guessed_id = detector.query(test_scene, classifier[ni], registration[ni]);

          guesses_for_id[guessed_id] += 1;
          confusion_matrix.increment(truth_id, guessed_id);
          cout << "detector guessed " << guessed_id << endl;
        } catch (exception &e) {
          cout << "Detector exception" << endl;
          cout << e.what() << endl;
          memset(classifier[ni], 0, sizeof(classifier[ni]));
          memset(registration[ni], 0, sizeof(registration[ni]));
        }
        timer.stop(DETECTOR_TEST);

        cout << endl;
      }

      printConfusionMatrix(confusion_matrix);

      return confusion_matrix.trace();
    }

    typedef struct {
      int ni;
      int mi;
      double distance;
    } Detection;

    bool
    operator<(const Detection &a, const Detection &b)
    {
      return a.distance < b.distance;
    }

    void
    Proctor::printPrecisionRecall()
    {
      //vector<Detection> detections;
      //Detection d;
      //for (d.ni = 0; d.ni < Config::num_trials; d.ni++) {
        //for (d.mi = 0; d.mi < Config::num_models; d.mi++) {
          //d.distance = registration[d.ni][d.mi];
          //if (!d.distance) continue; // did not attempt registration on this model
          //detections.push_back(d);
        //}
      //}
      //sort(detections.begin(), detections.end());
      //int correct = 0;
      //for (unsigned int di = 0; di < detections.size(); di++) {
        //if (detections[di].mi == scenes[detections[di].ni].mi) {
          //correct++;
          //printf(
            //"%.6f %.6f %g\n",
            //double(correct) / double(di + 1),
            //double(correct) / double(Config::num_trials),
            //detections[di].distance
          //);
        //}
      //}
    }

    void
    Proctor::printClassifierStats()
    {
      //float avg = 0; // average rank of correct id
      //int area = 0; // area under curve of cumulative histogram
      //for (int ni = 0; ni < Config::num_trials; ni++) {
        //int answer = scenes[ni].mi;
        //float votes = classifier[ni][answer];
        //// figure out the rank in this trial
        //int rank = 1;
        //int tie = 0;
        //for (int mi = 0; mi < Config::num_models; mi++) {
          //if (classifier[ni][mi] > votes) rank++;
          //else if (classifier[ni][mi] == votes) tie++;
        //}
        //// contribute to average rank
        //avg += rank + float(tie) / 2;
        //// contribute to area under curve
        //area += Config::num_models - rank + 1;
      //}
      //avg /= Config::num_trials;
      //printf("average vote rank of correct model:                    %0.2f\n", avg);
      //printf("area under cumulative histogram of correct model rank: %d\n", area);
    }

    void
    Proctor::printTimer()
    {
      printf(
        "obtain training clouds: %10.3f sec\n"
        "obtain testing clouds:  %10.3f sec\n"
        "detector training:      %10.3f sec\n"
        "detector testing:       %10.3f sec\n",
        timer[OBTAIN_CLOUD_TRAINING],
        timer[OBTAIN_CLOUD_TESTING],
        timer[DETECTOR_TRAIN],
        timer[DETECTOR_TEST]
      );
    }

    void
    Proctor::printResults(Detector &detector)
    {
      // precision-recall
      printf("[precision-recall]\n");
      printPrecisionRecall();

      // classifier stats
      printf("[classifier stats]\n");
      printClassifierStats();

      // timing
      printf("[timing]\n");
      printTimer();
      printf("[detector timing]\n");
      detector.printTimer();
    }

    void
    Proctor::printConfusionMatrix(ConfusionMatrix &matrix)
    {
      // correct percentage
      printf("[overview]\n");
      printf("%d of %d correct (%.2f%%)\n", matrix.trace(), matrix.total(), float(matrix.trace()) / matrix.total() * 100);

      cout << endl;

      cout << "[confusion matrix]" << endl;
      matrix.printMatrix();

      cout << endl;
    }
  }


}
