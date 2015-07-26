/* Small wrapper around GTK+
 *
 * Copyright (C) 2014 Roland Philippsen. All rights reserved.
 *
 * BSD license:
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of
 *    contributors to this software may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHORS AND CONTRIBUTORS ``AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR THE CONTRIBUTORS TO THIS SOFTWARE BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
   \file gfx.hpp

   Simplified GUI wrapper around GTK, GDK, and Cairo.  By using this
   wrapper, you can easily create a custom graphical output with a few
   buttons that will be displayed along the bottom of the GUI window.
   Processing mouse events like clicking and dragging is also kept
   very simple.

   The basic approach is that you register various callback functions
   via function pointers.  Thus, for each custom button, you specify a
   separate function to gfx::add_button.  This function will
   get called when the corresponding button is clicked.  Similarly,
   your custom mouse callback is specified as a function pointer to
   the gfx::main.  Your custom drawing function gets
   registered there as well.  Various functions are provided so that
   you can draw shapes on the GUI.

   The gfx::main function gets called as the last action in
   your program.  it sets up GTK for you and enters its event handling
   loop.
*/

#ifndef CHOMP_HPP
#define CHOMP_HPP

namespace chomp{
typedef Eigen::VectorXd Vector;
typedef Eigen::MatrixXd Matrix;
typedef Eigen::Isometry3d Transform;
void run_chomp(Vector const &qs,Vector const &qe, Vector &xi,Matrix const &obs);
}
#endif // CHOMP_HPP
