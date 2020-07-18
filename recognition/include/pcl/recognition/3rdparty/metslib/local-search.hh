// METS lib source file - local-search.hh                   -*- C++ -*-
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

#ifndef LOCAL_SEARCH_HH_
#define LOCAL_SEARCH_HH_

namespace mets {
  /// @defgroup local_search Local Search
  /// @{

  /// @brief Local search algorithm. 
  ///
  /// With customary phase alternation
  /// and move managers generated neighborhood this can be used to do
  /// also a Random Restart Local Search, a Greedy Search,
  /// an Iterated Local Search and a Variable Neighborhood Search.
  template<typename move_manager_type>
  class local_search : public mets::abstract_search<move_manager_type>
  {
  public:
    /// @brief Creates a local search instance
    ///
    /// @param working The working solution (this will be modified
    /// during search) 
    ///
    /// @param best_so_far A different solution
    /// instance used to store the best solution found
    ///
    /// @param moveman A problem specific implementation of the
    /// move_manager_type concept used to generate the neighborhood.
    ///
    /// @param short_circuit Wether the search should stop on
    /// the first improving move or not.
    local_search(evaluable_solution& starting_point,
		 solution_recorder& recorder,
		 move_manager_type& moveman,
		 gol_type epsilon = 1e-7,
		 bool short_circuit = false);

    /// purposely not implemented (see Effective C++)
    local_search(const local_search&);
    local_search& operator=(const local_search&);
    
    /// @brief This method starts the local search process.
    ///
    /// To have a real local search you should provide an 
    /// move_manager_type than enumerates all feasible
    /// moves.
    ///
    virtual void
    search();

  protected:
    bool short_circuit_m;
    gol_type epsilon_m;
  };

  /// @}
  
}

template<typename move_manager_t>
mets::local_search<move_manager_t>::local_search(evaluable_solution& working,
						 solution_recorder& recorder,
						 move_manager_t& moveman,
						 gol_type epsilon,
						 bool short_circuit)
  : abstract_search<move_manager_t>(working, recorder, moveman),
    short_circuit_m(short_circuit), epsilon_m(epsilon)
{ 
  using base_t = abstract_search<move_manager_t>;
  base_t::step_m = 0; 
}

template<typename move_manager_t>
void
mets::local_search<move_manager_t>::search()
{
  using base_t = abstract_search<move_manager_t>;
  typename move_manager_t::iterator best_movit;

  base_t::solution_recorder_m.accept(base_t::working_solution_m);

  gol_type best_cost = 
    static_cast<mets::evaluable_solution&>(base_t::working_solution_m)
    .cost_function();

  do
    {
      base_t::moves_m.refresh(base_t::working_solution_m);
      best_movit = base_t::moves_m.end();
      for(typename move_manager_t::iterator movit = base_t::moves_m.begin();
	  movit != base_t::moves_m.end(); ++movit)
	{
	  // evaluate the cost after the move
	  gol_type cost = (*movit)->evaluate(base_t::working_solution_m);
	  if(cost < best_cost - epsilon_m)
	    {
	      best_cost = cost;
	      best_movit = movit;
	      if(short_circuit_m) break;
	    }
	} // end for each move
      
      if(best_movit != base_t::moves_m.end()) 
	{
	  (*best_movit)->apply(base_t::working_solution_m);
	  base_t::solution_recorder_m.accept(base_t::working_solution_m);
	  base_t::current_move_m = best_movit;
	  this->notify();
	}
      
    } while(best_movit != base_t::moves_m.end());
}
#endif
