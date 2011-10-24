template <int bins>
Timer<bins>::Timer() {
  memset(total, 0, sizeof(total));
}

template <int bins>
void Timer<bins>::start() {
  mark = clock();
}

template <int bins>
void Timer<bins>::stop(int bin) {
  clock_t now = clock();
  total[bin] += now - mark;
}

template <int bins>
double Timer<bins>::operator[](int bin) {
  return double(total[bin]) / CLOCKS_PER_SEC;
}
