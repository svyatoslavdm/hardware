//
// blocking_udp_client.cpp
// ~~~~~~~~~~~~~~~~~~~~~~~
//
// Copyright (c) 2003-2012 Christopher M. Kohlhoff (chris at kohlhoff dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//

#include <boost/asio/deadline_timer.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/asio/ip/udp.hpp>
#include <cstdlib>
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <iostream>

using boost::asio::deadline_timer;
using boost::asio::ip::udp;

namespace kuka_hardware
{
//----------------------------------------------------------------------

//
// This class manages socket timeouts by applying the concept of a deadline.
// Each asynchronous operation is given a deadline by which it must complete.
// Deadlines are enforced by an "actor" that persists for the lifetime of the
// client object:
//
//  +----------------+
//  |                |
//  | check_deadline |<---+
//  |                |    |
//  +----------------+    | async_wait()
//              |         |
//              +---------+
//
// If the actor determines that the deadline has expired, any outstanding
// socket operations are cancelled. The socket operations themselves are
// implemented as transient actors:
//
//   +---------------+
//   |               |
//   |    receive    |
//   |               |
//   +---------------+
//           |
//  async_-  |    +----------------+
// receive() |    |                |
//           +--->| handle_receive |
//                |                |
//                +----------------+
//
// The client object runs the io_service to block thread execution until the
// actor completes.
//
class udp_client
{
public:
  udp_client(const udp::endpoint& listen_endpoint)
    : socket_(io_service_, listen_endpoint),
      deadline_(io_service_)
  {
    // No deadline is required until the first socket operation is started. We
    // set the deadline to positive infinity so that the actor takes no action
    // until a specific deadline is set.
    deadline_.expires_at(boost::posix_time::pos_infin);

    // Start the persistent actor that checks for deadline expiry.
    check_deadline();
  }

  std::size_t receive(const boost::asio::mutable_buffer& buffer,
      boost::posix_time::time_duration timeout,
      boost::asio::ip::udp::endpoint& sender_endpoint, boost::system::error_code& ec)
  {
    // Set a deadline for the asynchronous operation.
    deadline_.expires_from_now(timeout);

    // Set up the variables that receive the result of the asynchronous
    // operation. The error code is set to would_block to signal that the
    // operation is incomplete. Asio guarantees that its asynchronous
    // operations will never fail with would_block, so any other value in
    // ec indicates completion.
    ec = boost::asio::error::would_block;
    std::size_t length = 0;

    // Start the asynchronous operation itself. The handle_receive function
    // used as a callback will update the ec and length variables.
    socket_.async_receive_from(boost::asio::buffer(buffer),
        sender_endpoint,
        boost::bind(&udp_client::handle_receive, _1, _2, &ec, &length));

    // Block until the asynchronous operation has completed.
    do io_service_.run_one(); while (ec == boost::asio::error::would_block);

    return length;
  }

  std::size_t receive(const boost::asio::mutable_buffer& buffer,
      boost::asio::ip::udp::endpoint& sender_endpoint, boost::system::error_code& ec)
  {
    size_t length = socket_.receive_from(boost::asio::buffer(buffer), sender_endpoint, 0, ec);

    return length;
  }

  template <typename ConstBufferSequence>
  void send(const ConstBufferSequence& buffer, boost::asio::ip::udp::endpoint& destination)
  {
    socket_.send_to(buffer, destination);
  }

private:
  void check_deadline()
  {
    // Check whether the deadline has passed. We compare the deadline against
    // the current time since a new asynchronous operation may have moved the
    // deadline before this actor had a chance to run.
    if (deadline_.expires_at() <= deadline_timer::traits_type::now())
    {
      // The deadline has passed. The outstanding asynchronous operation needs
      // to be cancelled so that the blocked receive() function will return.
      //
      // Please note that cancel() has portability issues on some versions of
      // Microsoft Windows, and it may be necessary to use close() instead.
      // Consult the documentation for cancel() for further information.
      socket_.cancel();

      // There is no longer an active deadline. The expiry is set to positive
      // infinity so that the actor takes no action until a new deadline is set.
      deadline_.expires_at(boost::posix_time::pos_infin);
    }

    // Put the actor back to sleep.
    deadline_.async_wait(boost::bind(&udp_client::check_deadline, this));
  }

  static void handle_receive(
      const boost::system::error_code& ec, std::size_t length,
      boost::system::error_code* out_ec, std::size_t* out_length)
  {
    *out_ec = ec;
    *out_length = length;
  }

private:
  boost::asio::io_service io_service_;
  udp::socket socket_;
  deadline_timer deadline_;
};

}