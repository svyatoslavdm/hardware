/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2014, Igor Kalevatykh, Bauman Moscow State Technical University
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
#ifndef REAL_TIME_BOX_H_
#define REAL_TIME_BOX_H_

namespace kawasaki_hardware
{

template<typename T, size_t BufferSize = 32>
  class RealtimeBox
  {
  public:
    RealtimeBox() :
        set_index_(-1), get_index_(-1)
    {
    }

    bool set(const T & value)
    {
      int index = next(set_index_);

      for(register int i=0; i < 8; ++i)
      {
        if(index != get_index_)
        {
          ring_[index] = value;
          set_index_ = index;
          return true;
        }
        index = next(index);
      }

      return false;
    }

    bool get(T & value)
    {
      if (set_index_ > -1)
      {
        get_index_ = set_index_;
        value = ring_[get_index_];
        return true;
      }

      return false;
    }

  private:
    int next(int current)
    {
      return (current + 1) % BufferSize;
    }

    T ring_[BufferSize];
    volatile int set_index_;
    volatile int get_index_;
  };
}

#endif /* REAL_TIME_BOX_H_ */
