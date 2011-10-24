#include <cstdio>

#include <vtkCellArray.h>
#include <vtkFloatArray.h>
#include <vtkPoints.h>

#include "proctor/proctor.h"

Proctor::Model Proctor::models[Config::num_models];

IndicesPtr Proctor::randomSubset(int n, int r) {
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

void Proctor::readModels(const char *base, int max_models, unsigned int seed) {
  srand(seed);
  IndicesPtr model_subset = randomSubset(max_models, Config::num_models);
  for (int mi = 0; mi < Config::num_models; mi++) {
    int id = (*model_subset)[mi];
    char path[256];
    FILE *file;
    int vertices, faces, edges;

    // read mesh
    models[mi].id = id;
    snprintf(path, sizeof(path), "%s/%d/m%d/m%d.off", base, id / 100, id, id);
    file = fopen(path, "r");
    int line = 1;

    // read header
    if (fscanf(file, "OFF\n%d %d %d\n", &vertices, &faces, &edges) != 3) {
      cerr << "invalid OFF header in file " << path << endl;
      exit(-1);
    } else {
      line += 2;
    }

    // read vertices
    vtkFloatArray *fa = vtkFloatArray::New();
    fa->SetNumberOfComponents(3);
    float (*v)[3] = reinterpret_cast<float (*)[3]>(fa->WritePointer(0, 3 * vertices));
    for (int j = 0; j < vertices; j++) {
      if (fscanf(file, "%f %f %f\n", &v[j][0], &v[j][1], &v[j][2]) != 3) {
        cerr << "invalid vertex in file " << path << " on line " << line << endl;
        exit(-1);
      } else {
        ++line;
      }
    }
    vtkPoints *p = vtkPoints::New();
    p->SetData(fa); fa->Delete();

    // read faces
    vtkCellArray *ca = vtkCellArray::New();
    vtkIdType (*f)[4] = reinterpret_cast<vtkIdType (*)[4]>(ca->WritePointer(faces, 4 * faces));
    for (int j = 0; j < faces; j++) {
      f[j][0] = 3; // only supports triangles...
      if (fscanf(file, "3 %lld %lld %lld\n", &f[j][1], &f[j][2], &f[j][3]) != 3) {
        cerr << "invalid face in file " << path << " on line " << line << endl;
        exit(-1);
      } else {
        ++line;
      }
    }

    fclose(file);

    models[mi].mesh = vtkPolyData::New(); // lives forever
    models[mi].mesh->SetPoints(p); p->Delete();
    models[mi].mesh->SetPolys(ca); ca->Delete();

    // read metadata
    snprintf(path, sizeof(path), "%s/%d/m%d/m%d_info.txt", base, id / 100, id, id);
    file = fopen(path, "r");
    while (!feof(file)) {
      char buf[256];
      fgets(buf, sizeof(buf), file);
      if (!strncmp("center: ", buf, 8)) {
        if (sscanf(buf, "center: (%f,%f,%f)\n", &models[mi].cx, &models[mi].cy, &models[mi].cz) != 3) {
          cerr << "invalid centroid in file " << path << endl;
          cerr << buf;
          exit(-1);
        }
      } else if (!strncmp("scale: ", buf, 7)) {
        if (sscanf(buf, "scale: %f\n", &models[mi].scale) != 1) {
          cerr << "invalid scale in file " << path << endl;
          cerr << buf;
          exit(-1);
        }
      }
    } // end while over info lines
    fclose(file);
  } // end for over models
}

PointCloud<PointNormal>::Ptr
Proctor::getFullPointCloud(int mi) {
    PointCloud<PointNormal>::Ptr full_cloud(new PointCloud<PointNormal>());

    for (int ti = 0; ti < theta_count; ti++) {
      for (int pi = 0; pi < phi_count; pi++) {
        *full_cloud += *Scanner::getCloudCached(mi, ti, pi);
        flush(cout << '.');
      }
    }

    return full_cloud;
}

void Proctor::train(Detector &detector) {
  cout << "[models]" << endl;
  timer.start();
  PointCloud<PointNormal>::Ptr clouds[Config::num_models];
  for (int mi = 0; mi < Config::num_models; mi++) {
    clouds[mi] = getFullPointCloud(mi);
    cout << " finished model " << mi << " (" << models[mi].id << ")" << endl;
  }
  timer.stop(OBTAIN_CLOUD_TRAINING);

  cout << "[training]" << endl;
  timer.start();
  detector.train(clouds);
  timer.stop(DETECTOR_TRAIN);
}

void Proctor::test(Detector &detector, unsigned int seed) {
  srand(seed);
  const float theta_scale = (theta_max - theta_min) / RAND_MAX;
  const float phi_scale = (phi_max - phi_min) / RAND_MAX;

  // prepare test vectors in advance
  for (int ni = 0; ni < Config::num_trials; ni++) {
    scenes[ni].mi = rand() % Config::num_models;
    scenes[ni].theta = theta_min + rand() * theta_scale;
    scenes[ni].phi = phi_min + rand() * phi_scale;
  }

  // run the tests
  trace = 0;
  memset(confusion, 0, sizeof(confusion));
  for (int ni = 0; ni < Config::num_trials; ni++) {
    cout << "[test " << ni << "]" << endl;
    timer.start();
    PointCloud<PointNormal>::Ptr scene = Scanner::getCloud(scenes[ni]);
    timer.stop(OBTAIN_CLOUD_TESTING);
    cout << "scanned model " << scenes[ni].mi << endl;

    timer.start();
    int guess;
    try {
      guess = detector.query(scene, classifier[ni], registration[ni]);
    } catch (exception &e) {
      cout << "Detector exception" << endl;
      cout << e.what() << endl;
      guess = 0;
      memset(classifier[ni], 0, sizeof(classifier[ni]));
      memset(registration[ni], 0, sizeof(registration[ni]));
    }
    timer.stop(DETECTOR_TEST);
    cout << "detector guessed " << guess << endl;

    confusion[scenes[ni].mi][guess]++;
    if (guess == scenes[ni].mi) trace++;
  }
}

typedef struct {
  int ni;
  int mi;
  double distance;
} Detection;

bool operator<(const Detection &a, const Detection &b) {
  return a.distance < b.distance;
}

void Proctor::printPrecisionRecall() {
  vector<Detection> detections;
  Detection d;
  for (d.ni = 0; d.ni < Config::num_trials; d.ni++) {
    for (d.mi = 0; d.mi < Config::num_models; d.mi++) {
      d.distance = registration[d.ni][d.mi];
      if (!d.distance) continue; // did not attempt registration on this model
      detections.push_back(d);
    }
  }
  sort(detections.begin(), detections.end());
  int correct = 0;
  for (int di = 0; di < detections.size(); di++) {
    if (detections[di].mi == scenes[detections[di].ni].mi) {
      correct++;
      printf(
        "%.6f %.6f %g\n",
        double(correct) / double(di + 1),
        double(correct) / double(Config::num_trials),
        detections[di].distance
      );
    }
  }
}

void Proctor::printClassifierStats() {
  float avg = 0; // average rank of correct id
  int area = 0; // area under curve of cumulative histogram
  for (int ni = 0; ni < Config::num_trials; ni++) {
    int answer = scenes[ni].mi;
    float votes = classifier[ni][answer];
    // figure out the rank in this trial
    int rank = 1;
    int tie = 0;
    for (int mi = 0; mi < Config::num_models; mi++) {
      if (classifier[ni][mi] > votes) rank++;
      else if (classifier[ni][mi] == votes) tie++;
    }
    // contribute to average rank
    avg += rank + float(tie) / 2;
    // contribute to area under curve
    area += Config::num_models - rank + 1;
  }
  avg /= Config::num_trials;
  printf("average vote rank of correct model:                    %0.2f\n", avg);
  printf("area under cumulative histogram of correct model rank: %d\n", area);
}

void Proctor::printTimer() {
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

void Proctor::printResults(Detector &detector) {
  // correct percentage
  printf("[overview]\n");
  printf("%d of %d correct (%.2f%%)\n", trace, Config::num_trials, float(trace) / Config::num_trials * 100);

  // precision-recall
  printf("[precision-recall]\n");
  printPrecisionRecall();

  // classifier stats
  printf("[classifier stats]\n");
  printClassifierStats();

  // confusion matrix
  printf("[confusion matrix]\n");
  for (int i = 0; i < Config::num_models; i++) {
    for (int j = 0; j < Config::num_models; j++) {
      printf(" %3d", confusion[i][j]);
    }
    printf("\n");
  }

  // timing
  printf("[timing]\n");
  printTimer();
  printf("[detector timing]\n");
  detector.printTimer();
}
