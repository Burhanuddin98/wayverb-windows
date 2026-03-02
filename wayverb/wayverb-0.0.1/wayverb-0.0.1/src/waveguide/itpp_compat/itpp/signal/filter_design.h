#pragma once
// Drop-in replacement for itpp::vec and itpp::yulewalk
// Implements only what wayverb uses from itpp/signal/filter_design.h

#include <algorithm>
#include <cmath>
#include <complex>
#include <stdexcept>
#include <string>
#include <vector>

namespace itpp {

/// Minimal vec class matching the itpp::vec interface used by wayverb
class vec {
public:
    vec() = default;
    explicit vec(int n) : data_(n, 0.0) {}

    double& operator[](int i) { return data_[i]; }
    const double& operator[](int i) const { return data_[i]; }
    int size() const { return static_cast<int>(data_.size()); }

    std::vector<double> data_;
};

namespace detail {

/// Linear interpolation within sorted (x, y) data
static double interp1(const vec& fx, const vec& fy, double x) {
    if (x <= fx[0]) return fy[0];
    if (x >= fx[fx.size() - 1]) return fy[fy.size() - 1];
    int lo = 0, hi = fx.size() - 1;
    while (hi - lo > 1) {
        int mid = (lo + hi) / 2;
        if (fx[mid] <= x) lo = mid; else hi = mid;
    }
    double t = (x - fx[lo]) / (fx[hi] - fx[lo] + 1e-15);
    t = std::max(0.0, std::min(1.0, t));
    return fy[lo] + t * (fy[hi] - fy[lo]);
}

/// Levinson-Durbin recursion.
/// Solves the symmetric Toeplitz system built from r[0..N].
/// Returns AR coefficients a[0..N] with a[0] = 1.
static std::vector<double> levinson(const std::vector<double>& r, int N) {
    std::vector<double> a(N + 1, 0.0), tmp(N + 1, 0.0);
    a[0] = 1.0;
    double err = r[0];
    if (err <= 0.0) return a;

    for (int m = 1; m <= N; ++m) {
        double k = r[m];
        for (int j = 1; j < m; ++j) k += a[j] * r[m - j];
        k = -k / err;
        tmp[0] = 1.0;
        for (int j = 1; j < m; ++j) tmp[j] = a[j] + k * a[m - j];
        tmp[m] = k;
        a = tmp;
        err *= (1.0 - k * k);
        if (err <= 0.0) { err = 1e-30; break; }
    }
    return a;
}

} // namespace detail

/// Yule-Walker IIR filter design from an arbitrary magnitude response.
///
/// n    : filter order (b and a will each have n+1 coefficients)
/// f    : frequency vector, normalized 0..1 (0 = DC, 1 = Nyquist)
/// m    : desired magnitude at each frequency in f
/// b    : output MA (numerator) coefficients, length n+1
/// a    : output AR (denominator) coefficients, length n+1, a[0]=1
inline void yulewalk(int n, const vec& f, const vec& m, vec& b, vec& a) {
    // 1. Interpolate desired magnitude to a uniform grid of npt points
    const int npt = 512;
    const int L = 2 * (npt - 1);   // symmetric DFT length

    std::vector<double> H(npt);
    for (int i = 0; i < npt; ++i) {
        double fi = static_cast<double>(i) / (npt - 1);
        H[i] = std::max(0.0, detail::interp1(f, m, fi));
    }

    // 2. Build symmetric power spectrum P[0..L-1]
    std::vector<double> P(L, 0.0);
    for (int i = 0; i < npt; ++i)      P[i]     = H[i] * H[i];
    for (int i = 1; i < npt - 1; ++i) P[L - i] = P[i];

    // 3. Autocorrelation of desired power spectrum via cosine transform.
    //    r[k] = (1/L) * Re{ sum_n P[n] * exp(j*2*pi*n*k/L) }
    //    Since P is real and symmetric this reduces to the cosine sum.
    std::vector<double> r(n + 2, 0.0);
    for (int k = 0; k <= n + 1; ++k) {
        double s = P[0];
        for (int nn = 1; nn < L; ++nn)
            s += P[nn] * std::cos(2.0 * M_PI * nn * k / L);
        r[k] = s / L;
    }

    // 4. Solve Yule-Walker for AR (denominator) coefficients
    std::vector<double> a_coef = detail::levinson(r, n);

    // 5. Compute the MA (numerator) coefficients.
    //    Desired |B(w)|^2 = |H_desired(w)|^2 * |A(w)|^2.
    //    Build its autocorrelation, then fit an AR model to approximate B.

    // Evaluate |A(e^jw)|^2 at all L frequency points
    std::vector<double> A2(L, 0.0);
    for (int i = 0; i < L; ++i) {
        double w = 2.0 * M_PI * i / L;
        std::complex<double> Az(0.0, 0.0);
        for (int k = 0; k <= n; ++k)
            Az += a_coef[k] * std::polar(1.0, -k * w);
        A2[i] = std::norm(Az);
    }

    // PB[i] = desired MA power spectrum
    std::vector<double> PB(L);
    for (int i = 0; i < L; ++i)
        PB[i] = P[i] * A2[i];

    // Autocorrelation of MA power spectrum
    std::vector<double> rb(n + 1, 0.0);
    for (int k = 0; k <= n; ++k) {
        double s = PB[0];
        for (int nn = 1; nn < L; ++nn)
            s += PB[nn] * std::cos(2.0 * M_PI * nn * k / L);
        rb[k] = s / L;
    }

    // Levinson on rb gives an AR approximation to the MA polynomial
    std::vector<double> b_coef = detail::levinson(rb, n);

    // Scale so that b[0] reflects the correct power level
    double gain = (rb[0] > 0.0) ? std::sqrt(rb[0]) : 1.0;
    for (auto& v : b_coef) v *= gain;

    // 6. Pack into output vecs
    b = vec(n + 1);
    a = vec(n + 1);
    for (int i = 0; i <= n; ++i) {
        b[i] = b_coef[i];
        a[i] = a_coef[i];
    }
}

} // namespace itpp
