#pragma once

const int nr_threads = 1;
const float approx_overlap = 0.9f;
const float delta = 1.f;
const int nr_samples = 100;

const float transform_from_fpcs [4][4] = {
  { -0.0019f, 0.8266f, -0.5628f, -0.0378f },
  { -0.9999f, -0.0094f, -0.0104f, 0.9997f },
  { -0.0139f, 0.5627f, 0.8265f, 0.0521f },
  { 0.f, 0.f, 0.f, 1.f }
};
