#pragma once

#include <array>
#include <mutex>

template <typename T, std::size_t S>
class CircularBuffer
{
public:
  CircularBuffer() {}

  bool put(const T *buf, size_t len)
  {
    std::unique_lock lock(d_mutex);

    size_t available = __available();

    if (available < len) {
      return false;
    }

    if (len == available) {
      d_full = true;
    }
    
    if (d_head + len >= d_size) {
      size_t cycle = d_size - d_head;
      for (size_t i = 0; i < cycle; ++i) {
        d_data[d_head + i] = buf[i];
      }
      for (size_t i = 0; i < len - cycle; ++i) {
        d_data[i] = buf[i + cycle];
      }
      d_head = len - cycle;
    } else {
      for (size_t i = 0; i < len; ++i) {
        d_data[d_head + i] = buf[i];
      }
      d_head += len;
    }

    return true;
  }
  bool get(size_t num, T *buf)
  {
    std::unique_lock lock(d_mutex);
    if (__used() < num) {
      return false;
    }

    if (num > 0) {
      d_full = false;
    }

    if (d_tail + num >= d_size) {
      size_t cycle = d_size - d_tail;

      for (size_t i = 0; i < cycle; ++i) {
        buf[i] = d_data[d_tail + i];
      }
      for (size_t i = 0; i < num - cycle; ++i) {
        buf[i + cycle] = d_data[i];
      }
      d_tail = num - cycle;
    } else {
      for (size_t i = 0; i < num; ++i) {
        buf[i] = d_data[d_tail + i];
      }
      d_tail += num;
    }

    return true;
  }
  bool peek(size_t num, T *buf)
  {
    std::unique_lock lock(d_mutex);
    if (__used() < num) {
      return false;
    }

    if (d_tail + num >= d_size) {
      size_t cycle = d_size - d_tail;

      for (size_t i = 0; i < cycle; ++i) {
        buf[i] = d_data[d_tail + i];
      }
      for (size_t i = 0; i < num - cycle; ++i) {
        buf[i + cycle] = d_data[i];
      }
    } else {
      for (size_t i = 0; i < num; ++i) {
        buf[i] = d_data[d_tail + i];
      }
    }

    return true;
  }

  size_t size()
  {
    return d_size;
  }
  size_t available()
  {
    std::unique_lock lock(d_mutex);
    return __available();
  }
  size_t used()
  {
    std::unique_lock lock(d_mutex);
    return __used();
  }
  bool empty()
  {
    return used() == 0;
  }
  bool full()
  {
    return d_full;
  }

  void reset()
  {
    std::unique_lock lock(d_mutex);
    d_tail = d_head = 0;
    d_full = 0;
  }

protected:
  inline size_t __available() const
  {
    if (d_full) {
      return 0;
    }

    if (d_tail > d_head) {
      return d_tail - d_head;
    } else {
      return d_size - d_head + d_tail;
    }
  }
  inline size_t __used() const
  {
    return d_size - __available();
  }

protected:
  std::array<T, S> d_data;
  std::mutex d_mutex;
  bool d_full = false;
  const size_t d_size = S;
  size_t d_head = 0;
  size_t d_tail = 0;

};
