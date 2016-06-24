/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2014-2015, Bauman Moscow State Technical University
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *      * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *      * Neither the name of the Bauman Moscow State Technical University,
 *      nor the names of its contributors may be used to endorse or promote
 *      products derived from this software without specific prior written
 *      permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/// \author Igor Kalevatykh <kalevatykhia@gmail.com>

#ifndef KAWASAKI_HARDWARE__BUFFER_H
#define KAWASAKI_HARDWARE__BUFFER_H

#include <boost/atomic.hpp>

namespace kawasaki_hardware
{

template<typename T, size_t Size = 3>
class Buffer
{
public:
  Buffer() :
      wrote_(0), read_(0)
  {
  }

  bool set(const T & value)
  {
    size_t wrote = wrote_.load(boost::memory_order_relaxed);
    size_t write = next(wrote);
    while (write == read_.load(boost::memory_order_acquire))
    {
      write = next(write);
      if (write == wrote) { return false; }
    }

    buffer_[write] = value;
    wrote_.store(write, boost::memory_order_release);
    return true;
  }

  void get(T & value)
  {
    size_t wrote = wrote_.load(boost::memory_order_acquire);
    read_.store(wrote, boost::memory_order_release);
    value = buffer_[wrote];
  }

private:
  size_t next(size_t current)
  {
    return (current + 1) % Size;
  }
  T buffer_[Size];
  boost::atomic_size_t wrote_, read_;
};

}

#endif /* KAWASAKI_HARDWARE__BUFFER_H */
