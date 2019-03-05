std::vector<double> linDoubleSpace(double min, double max, size_t N) {
  std::vector<double> range;
  double delta = (max-min)/double(N-1);
  for (int i = 0; i < N; i++) {
    double curValue = min + i*delta;
    range.push_back(curValue);
  }
  return range;
}

std::vector<int> linIntSpace(double min, double max, size_t N) {
  std::vector<int> intRange;
  std::vector<double> range = linDoubleSpace(min, max, N);
  for (double d : range) {
    intRange.push_back((int) d);
  }

  return intRange;
}
