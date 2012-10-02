// METS lib source file - mets.h                               -*- C++ -*-
//
/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2006-2012, Mirko Maischberger <mirko.maischberger@gmail.com>
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */
/// @mainpage METSlib
///
/// \image html http://www.coin-or.org/images/logo/COIN-OR_150.png
/// 
/// @section Introduction
///
/// This is a library implementing some neighborhood based
/// metaheuristics with or without memory.
///
/// The solution instance must implement mets::feasible_solution and
/// the moves must be derived from mets::move classes.
///
/// The neighborhood can be specified implementing a
/// mets::move_manager subclass or providing another class with the
/// same concept (using the move_manager can ease things, but you can
/// also provide your own custom neighorhood iterator).
///
/// All the mentioned classes must be implemented to model the problem
/// at hand.  See as an example the "tutorial" and "qap" programs.
///
/// You are also responsible of configuring and running the algorithm
/// of choice.
///
/// Once your model is set up you are free of experimenting different
/// metaheuristics without changing it, but simply configuring one
/// algorithm or another.
///
/// Each algorithm can be customized implementing your own decorating
/// classes, although a bunch of predefined and commonly used
/// decorators are provided with the library, some problems may need
/// customary tabu lists, termination criterias, aspiration criteria,
/// or cooling schedules.
/// 
/// The framework you must implement your model into is made of:
///
/// - mets::feasible_solution
///   - mets::evaluable_solution (use this if you also use mets::best_ever_solution)
///   - mets::permutation_problem
/// - mets::move
///   - mets::mana_move (use this if you also use by mets::simple_tabu_list)
///   - mets::swap_elements
///
/// The toolkit of implemented algorithms is made of:
///
/// - mets::move_manager (or a class implementing the same concept)
///   - mets::swap_neighborhood
/// - mets::local_search
/// - mets::simulated_annealing
///   - mets::abstract_cooling_schedule
///   - mets::solution_recorder
///     - mets::best_ever_solution
///   - mets::termination_criteria_chain
///     - mets::iteration_termination_criteria
///     - mets::noimprove_termination_criteria
///     - mets::threshold_termination_criteria
/// - mets::tabu_search
///   - mets::tabu_list_chain
///     - mets::simple_tabu_list
///   - mets::aspiration_criteria_chain
///     - mets::best_ever_criteria
///   - mets::solution_recorder
///     - mets::best_ever_solution
///   - mets::termination_criteria_chain
///     - mets::iteration_termination_criteria
///     - mets::noimprove_termination_criteria
///     - mets::threshold_termination_criteria
///
/// To use the mets::simple_tabu_list you need to derive your moves
/// from the mets::mana_move base class and implement the pure virtual
/// methods.
///
#ifndef METS_METS_HH_
#define METS_METS_HH_

#include "metslib_config.hh"

#include <list>
#include <cmath>
#include <deque>
#include <limits>
#include <string>
#include <vector>
#include <cassert>
#include <typeinfo>
#include <iostream>
#include <stdexcept>
#include <algorithm>

#if defined (METSLIB_TR1_BOOST)
#  include <boost/random/uniform_int.hpp>
#  include <boost/random/uniform_real.hpp>
#  include <boost/unordered_map.hpp>
#elif defined (METSLIB_HAVE_UNORDERED_MAP)
#  include <unordered_map>
#  include <random>
#elif defined (METSLIB_HAVE_TR1_UNORDERED_MAP)
#  include <tr1/unordered_map>
#  include <tr1/random>
#else
#  error "Unable to find unordered_map header file. Please use a recent C++ compiler supporting TR1 extension."
#endif


///
/// @brief METSlib Metaheuristic framework namespace.
///
/// Framework for neighborhood based metaheuristics (Tabu Search,
/// Simulated Annealing, Iterated Local Search, Random Restart Local
/// Search).
///

#include "observer.hh"
#include "model.hh"
#include "termination-criteria.hh"
#include "abstract-search.hh"
#include "local-search.hh"
#include "tabu-search.hh"
#include "simulated-annealing.hh"


//________________________________________________________________________
inline std::ostream& 
operator<<(std::ostream& os, const mets::printable& p)
{
  p.print(os);
  return os;
}

#endif
