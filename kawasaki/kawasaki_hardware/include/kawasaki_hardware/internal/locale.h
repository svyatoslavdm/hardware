/*
 Copyright (c) 2012-2013 Igor Kalevatykh, All Rights Reserved.

 This file is part of open-kawasaki (http://open-kawasaki.googlecode.com/).

 Open-kawasaki is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 Open-kawasaki is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with open-kawasaki. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef KAWASAKI_HARDWARE__LOCALE_H
#define KAWASAKI_HARDWARE__LOCALE_H

#include <locale>

namespace kawasaki_hardware
{

/** @brief Kawasaki industrial controllers numbers printing and scanning settings.
 *  @ingroup internal
 */
class numpunct : public std::numpunct<char>
{
protected:

  virtual char_type do_decimal_point() const
  {
    return '.';
  }

  virtual std::string do_grouping() const
  {
    return "";
  }

  virtual string_type do_truename( ) const
  {
      return "TRUE";
  }

  virtual string_type do_falsename( ) const
  {
      return "FALSE";
  }
};

/** @brief Kawasaki industrial controllers locale.
 *  @ingroup internal
 */
class controller_locale : public std::locale
{
public:
  controller_locale() :
    std::locale(std::locale(), new numpunct)
  {
  }
};

}

#endif /* KAWASAKI_HARDWARE__LOCALE_H */
