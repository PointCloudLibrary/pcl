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

#include <iostream>

#ifndef METS_ABSTRACT_SEARCH_HH_
#define METS_ABSTRACT_SEARCH_HH_

namespace mets {

  /// @defgroup common Common components
  /// @{

  /// @brief The solution recorder is used by search algorithm, at the
  /// end of each iteration, to record the best seen solution.
  /// 
  /// The concept of best is externalized so that you can record the
  /// best ever solution met or the best solution that matches some
  /// other criteria (e.g. feasibility constraints relaxed in the
  /// feasible_solution implementation of the cost function).
  ///
  class solution_recorder {
  public:
    /// @brief Default ctor.
    solution_recorder() {}
    /// @brief Unimplemented copy ctor.
    solution_recorder(const solution_recorder&);
    /// @brief Unimplemented assignment operator.
    solution_recorder& operator=(const solution_recorder&);

    /// @brief A virtual dtor.
    virtual 
    ~solution_recorder();

    /// @brief Accept is called at the end of each iteration for an
    /// opportunity to record the best move ever.
    ///
    /// (this is a chain of responsibility)
    ///
    virtual bool 
    accept(const feasible_solution& sol) = 0;

    virtual gol_type 
    best_cost() const = 0;
  };

  /// @brief An abstract search.
  ///
  /// @see mets::tabu_search, mets::simulated_annealing, mets::local_search
  template<typename move_manager_type>
  class abstract_search : public subject< abstract_search<move_manager_type> >
  {
  public:
    /// @brief Set some common values needed for neighborhood based
    /// metaheuristics.
    ///
    /// @param working The starting point solution (this will be modified
    /// during search as the working solution) 
    ///
    /// @param recorder A solution recorder instance used to record
    /// the best solution found
    ///
    /// @param moveman A problem specific implementation of the
    /// move_manager_type used to generate the neighborhood.
    ///
    abstract_search(feasible_solution& working,
		    solution_recorder& recorder,
		    move_manager_type& moveman)
      : subject<abstract_search<move_manager_type> >(), 
	solution_recorder_m(recorder),
	working_solution_m(working),
	moves_m(moveman),
	current_move_m(),
	step_m()
    { }
			 
    /// purposely not implemented (see Effective C++)
    abstract_search(const abstract_search<move_manager_type>&);
    /// purposely not implemented (see Effective C++)
    abstract_search& operator==(const abstract_search<move_manager_type>&);

    /// @brief Virtual destructor.
    virtual 
    ~abstract_search() 
    { };

    enum {
      /// @brief We just made a move.
      MOVE_MADE = 0,  
      /// @brief Our solution_recorder_chain object reported an improvement
      IMPROVEMENT_MADE,
      /// @brief We are about to start a new iteration
      ITERATION_BEGIN,
      /// @brief We have done the iteration
      ITERATION_END,
      /// @brief Placeholer for next values
      LAST
    };
     
    /// @brief This method starts the search.
    /// 
    /// Remember that this is a minimization.
    ///
    /// An exception mets::no_moves_error can be risen when no move is
    /// possible.
    virtual void
    search() = 0;

    /// @brief The solution recorder instance.
    const solution_recorder&
    recorder() const 
    { return solution_recorder_m; };
    
    /// @brief The current working solution.
    const feasible_solution&
    working() const 
    { return working_solution_m; }
    
    feasible_solution&
    working() 
    { return working_solution_m; }

    /// @brief The last move made
    const move&
    current_move() const 
    { return **current_move_m; }

    /// @brief The last move made
    move&
    current_move() 
    { return **current_move_m; }
 
    /// @brief The move manager used by this search
    const move_manager_type& 
    move_manager() const 
    { return moves_m; }

    /// @brief The move manager used by this search
    move_manager_type& 
    move_manager() 
    { return moves_m; }

    /// @brief The current step of the algorithm (to be used by the
    ///        observers).
    ///        
    /// When you implement a new type of search you should set step_m
    /// protected variable to the status of the algorithm
    /// (0 = "MOVE_MADE", 1 = "IMPROVEMENT_MADE", etc.).
    int
    step() const 
    { return step_m; }
    
  protected:
    solution_recorder& solution_recorder_m;
    feasible_solution& working_solution_m;
    move_manager_type& moves_m;
    typename move_manager_type::iterator current_move_m;
    int step_m;
  };

  /// @}

  /// @defgroup common Common components
  /// @{

  /// @brief The best ever solution recorder can be used as a simple
  /// solution recorder that just records the best copyable solution
  /// found during its lifetime.
  /// 
  class best_ever_solution : public solution_recorder 
  {
  public:
    /// @brief The mets::evaluable_solution will be stored as a
    /// reference: please provide an instance that is not
    /// modified/needed elsewhere.
    ///
    /// @param best The instance used to store the best solution found
    /// (will be modified).
    best_ever_solution(evaluable_solution& best) : 
      solution_recorder(), 
      best_ever_m(best) 
    { }

    /// @brief Unimplemented default ctor.
    best_ever_solution();
    /// @brief Unimplemented copy ctor.
    best_ever_solution(const best_ever_solution&);
    /// @brief Unimplemented assignment operator.
    best_ever_solution& operator=(const best_ever_solution&);

    /// @brief Accept is called at the end of each iteration for an
    /// opportunity to record the best solution found during the
    /// search.
    bool accept(const feasible_solution& sol);

    /// @brief Returns the best solution found since the beginning.
    const evaluable_solution& best_seen() const 
    { return best_ever_m; }

    /// @brief Best cost seen.
    gol_type best_cost() const 
    { return best_ever_m.cost_function(); }
  protected:
    /// @brief Records the best solution
    evaluable_solution& best_ever_m;
  };
    
  /// @brief An object that is called back during the search progress.
  template<typename move_manager_type>
  class search_listener : public observer<abstract_search<move_manager_type> >
  {
  public:
    using search_type = abstract_search<move_manager_type>;
    /// @brief A new observer (listener) of a search process, remember
    /// to attach the created object to the search process to be
    /// observed (mets::search_type::attach())
    explicit
    search_listener() : observer<search_type>() 
    { }

    /// purposely not implemented (see Effective C++)
    search_listener(const search_listener<search_type>& other);
    search_listener<search_type>& 
    operator=(const search_listener<search_type>& other);

    /// @brief Virtual destructor
    virtual 
    ~search_listener() 
    { }

    /// @brief This is the callback method called by searches
    /// when a move, an improvement or something else happens
    virtual void
    update(search_type* algorithm) = 0;
  };


  template<typename neighborhood_t>
  struct iteration_logger : public mets::search_listener<neighborhood_t>
  {
    explicit
    iteration_logger(std::ostream& o) 
      : mets::search_listener<neighborhood_t>(), 
	iteration(0), 
	os(o) 
    { }
    
    void 
    update(mets::abstract_search<neighborhood_t>* as) 
    {
      const mets::feasible_solution& p = as->working();
      if(as->step() == mets::abstract_search<neighborhood_t>::MOVE_MADE)
	{
	  os << iteration++ << "\t" 
	     << static_cast<const mets::evaluable_solution&>(p).cost_function()
	     << "\n";
	}
    }
    
  protected:
    int iteration;
    std::ostream& os;
  };

  template<typename neighborhood_t>
  struct improvement_logger : public mets::search_listener<neighborhood_t>
  {
    explicit
    improvement_logger(std::ostream& o, gol_type epsilon = 1e-7) 
      : mets::search_listener<neighborhood_t>(), 
	iteration_m(0), 
	best_m(std::numeric_limits<double>::max()),
	os_m(o),
	epsilon_m(epsilon)
    { }
    
    void 
    update(mets::abstract_search<neighborhood_t>* as) 
    {
      const mets::feasible_solution& p = as->working();

      if(as->step() == mets::abstract_search<neighborhood_t>::MOVE_MADE)
	{
	  iteration_m++;
	  double val = static_cast<const mets::evaluable_solution&>(p)
	    .cost_function();
	  if(val < best_m - epsilon_m) 
	    {	     
	      best_m = val;
	      os_m << iteration_m << "\t" 
		   << best_m
		   << " (*)\n";
	    }
	}
    }
    
  protected:
    int iteration_m;
    double best_m;
    std::ostream& os_m;
    gol_type epsilon_m;
  };

  /// @}


}

inline mets::solution_recorder::~solution_recorder() 
{ }

inline bool
mets::best_ever_solution::accept(const mets::feasible_solution& sol)
{
  const evaluable_solution& s = dynamic_cast<const mets::evaluable_solution&>(sol);
  if(s.cost_function() < best_ever_m.cost_function())
    {
      best_ever_m.copy_from(s);
      return true;
    }
  return false;
}

#endif
