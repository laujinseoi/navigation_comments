/*
 * Copyright (c) 2013, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
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

#include <global_planner/quadratic_calculator.h>

namespace global_planner {
float QuadraticCalculator::calculatePotential(float* potential, unsigned char cost, int n, float prev_potential) {

    // 获得n的四个位置的位置值
    //-------
    //|1|2|3|
    //-------
    //|4|5|6|
    //-------
    //|7|8|9|
    //-------
    //nx_为列数
    //以5为例子,则u = 5-3=2;
    //以此类推
    float u, d, l, r;
    l = potential[n - 1];
    r = potential[n + 1];
    u = potential[n - nx_];
    d = potential[n + nx_];
    //  ROS_INFO("[Update] c: %f  l: %f  r: %f  u: %f  d: %f\n",
    //     potential[n], l, r, u, d);
    //  ROS_INFO("[Update] cost: %d\n", costs[n]);

    // find lowest, and its lowest neighbor

    /*****选择是前进还是绕着当前点*****/
    /*Navfn uses Dijkstra's algorithm to search through the available
     *working area and find the best path. What this means is that navfn
     * uses breadth-first search to check the costmap cells for the optimal path.
     * It begins at the robot's origin and radiates outward, checking each
     * non-obstacle cell in turn until the goal is reached. You can find a primer
     * on Dijkstra's algorithm here.Navfn's optimal path is based on a path's "potential".
     * The potential is the relative cost of a path based on the distance from the goal and
     * from the existing path itself.
     * It must be noted that Navfn update's each cell's potential in the potential map, or potarr
     * as it's called in navfn, as it checks that cell. This way,
     * it can step back through the potential array to find the best possible path. The potential
     * is determined by the cost of traversing a cell (traversability factor, hf) and the distance
     * away that the next cell is from the previous cell. The section of code that you posted occurs
     * during the update of a given cell during traversal. The quadratic approximation is used to
     * overcome the limitation that navfn's potential array is four-connected, i.e.
     * it only checks it's top, right, left, and bottom neighbors and not the diagonals. For example,
     * in the section of code that you posted, tc is defined as the cell with the lowest of either the
     * left cell or the right cell, and ta is the lowest of the top cell or the bottom cell.
     *  It then calculates the relative cost between these two cells. Then, the algorithm checks to
     * see if the relative cost of traversing through a cell is less than the cost of going around it.
     * If the cost of going around a cell is less than going through it, the algorithm uses the quadratic
     * estimation to determine an approximate cost.
     * https://answers.ros.org/question/11388/navfn-algorism/?answer=16891#answer-container-16891
*/
    float ta, tc;
    if (l < r)
        tc = l;
    else
        tc = r;
    if (u < d)
        ta = u;
    else
        ta = d;

    float hf = cost; // traversability factor
    float dc = tc - ta;        // relative cost between ta,tc
    if (dc < 0)         // tc is lowest
            {
        dc = -dc;
        ta = tc;
    }

    //??????
    // calculate new potential
    if (dc >= hf)        // if too large, use ta-only update
        return ta + hf;
    else            // two-neighbor interpolation update
    {
        // use quadratic approximation
        // might speed this up through table lookup, but still have to
        //   do the divide
        float d = dc / hf;
        float v = -0.2301 * d * d + 0.5307 * d + 0.7040;
        return ta + hf * v;
    }
}
}

