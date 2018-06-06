/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted
 * provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright notice, this list of
 *       conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright notice, this list of
 *       conditions and the following disclaimer in the documentation and/or other materials
 *       provided with the distribution.
 *     * Neither the name of the Georgia Tech Research Corporation nor the names of its
 *       contributors may be used to endorse or promote products derived from this software without
 *       specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL GEORGIA TECH RESEARCH
 * CORPORATION BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 * USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/**
 * @file display.hpp
 * @author Saul Reynolds-Haertle
 * @date July 29, 2013
 * @brief Common code for displaying things with curses.
 */

#pragma once

#include <Eigen/Dense>
#include <ncurses.h>

namespace Krang {

extern int COLOR_RED_BACKGROUND;
extern int COLOR_YELLOW_BACKGROUND;
extern int COLOR_GREEN_BACKGROUND;
extern int COLOR_WHITE_BACKGROUND;

// some configuration and utility variables
extern int curses_display_precision;
extern int curses_display_row;
extern bool doing_curses;

// functions for starting up and shutting down curses cleanly
void init_curses();
void destroy_curses();

// functions for nicely formatted and aligned debug output
void curses_display_vector(const Eigen::VectorXd& v, const char* label = "", int column = 0, 
		int color = COLOR_WHITE);
void curses_display_matrix(const Eigen::MatrixXd& m, const char* label = "", int column = 0, 
		int color = COLOR_WHITE);
}
