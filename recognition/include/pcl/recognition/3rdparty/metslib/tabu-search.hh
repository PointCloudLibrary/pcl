// METSlib source file - tabu-search.hh                          -*- C++ -*-
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

#ifndef METS_TABU_SEARCH_HH_
#define METS_TABU_SEARCH_HH_

namespace mets {

  /// @defgroup tabu_search Tabu Search
  /// @{

  /// @brief Function object expressing an
  /// aspiration criteria
  ///
  /// An aspiration criteria is a criteria used
  /// to override the tabu list. When the aspiration
  /// criteria is met a move is made even if it's in 
  /// the tabu-list
  ///
  /// Aspiration critera can be chained so a criteria can decorate
  /// another criteria
  class aspiration_criteria_chain
  {
  public:
    /// @brief Constructor.
    /// 
    /// @param next Optional next criteria in the chain.
    explicit
    aspiration_criteria_chain(aspiration_criteria_chain *next = 0)
      : next_m(next) 
    { }
    
    /// purposely not implemented (see Effective C++)
    aspiration_criteria_chain(const aspiration_criteria_chain& other);
    /// purposely not implemented (see Effective C++)
    aspiration_criteria_chain& 
    operator=(const aspiration_criteria_chain& other);

    /// @brief Virtual destructor.
    virtual 
    ~aspiration_criteria_chain() 
    { } 

    /// @brief A method to reset this aspiration criteria chain to its
    /// original state.
    virtual void 
    reset();

    /// @brief This is a callback function from the algorithm that
    /// tells us that a move was accepted.
    ///
    /// You can use this function to update the aspiration criteria
    /// based on the current search status. (e.g. record the best cost
    /// for a best ever criteria)
    ///
    /// @param fs The current working solution (after applying move).
    /// @param mov The accepted move (the move just made).
    /// @return True if the move is to be accepted.
    virtual void 
    accept(feasible_solution& fs, move& mov, gol_type evaluation);
    
    /// @brief The function that decides if we shoud accept a tabu move
    ///
    /// @param fs The current working solution (before applying move).
    /// @param mov The move to be made (the move that is being evaluated).
    /// @return True if the move is to be accepted.
    virtual bool 
    operator()(feasible_solution& fs, move& mov, gol_type evaluation) const;
    
  protected:
    aspiration_criteria_chain* next_m;
  };

  ///
  /// @brief An abstract tabu list
  /// 
  /// This is chainable so that tabu lists can be decorated with
  /// other tabu lists.
  class tabu_list_chain
  {
  public:
    tabu_list_chain();
    /// purposely not implemented (see Effective C++)
    tabu_list_chain(const tabu_list_chain&);
    /// purposely not implemented (see Effective C++)
    tabu_list_chain& operator=(const tabu_list_chain&);

    /// Create an abstract tabu list with a certain tenure
    explicit
    tabu_list_chain(unsigned int tenure) 
      : next_m(0), tenure_m(tenure) 
    { }

    /// @brief Create an abstract tabu list with a certain tenure and
    /// a chained tabu list that decorates this one
    tabu_list_chain(tabu_list_chain* next, unsigned int tenure) 
      : next_m(next), tenure_m(tenure) 
    { }

    /// @brief Virtual destructor
    virtual 
    ~tabu_list_chain() 
    { } 

    ///
    /// @brief Make a move tabu when starting from a certain solution.
    /// 
    /// Different implementation can remember "tenure" moves, 
    /// "tenure" solutions or some other peculiar fact that
    /// will avoid cycling.
    ///
    /// Mind you! The solution here is the solution *before* applying
    /// the move: this is for efficiency reason.
    ///
    /// @param sol The current working solution
    /// @param mov The move to make tabu
    virtual void 
    tabu(feasible_solution& sol, move& mov) = 0;

    /// @brief True if the move is tabu for the given solution.
    ///
    /// Different implementation can remember "tenure" moves, 
    /// "tenure" solutions or some other peculiar fact that
    /// will avoid cycling. So it's not defined at this stage 
    /// if a move will be tabu or not at a certain state of the
    /// search: this depends on the implementation.
    ///
    /// Mind you! The solution here is the solution *before* applying
    /// the move: this is for efficiency reason.
    ///
    /// @param sol The current working solution
    /// @param mov The move to make tabu
    virtual bool 
    is_tabu(feasible_solution& sol, move& mov) const = 0;

    ///
    /// @brief Tenure of this tabu list.
    ///
    /// The tenure is the length of the tabu-list (the order of the
    /// tabu memory)
    ///
    virtual unsigned int
    tenure() const
    { return tenure_m; }

    ///
    /// @brief Tenure of this tabu list.
    ///
    /// @param tenure: the new tenure of the list.
    ///
    virtual void
    tenure(unsigned int tenure) 
    { tenure_m = tenure; }

  protected:
    tabu_list_chain* next_m;
    unsigned int tenure_m;
  };
  
  ///
  /// @brief Tabu Search algorithm.
  ///
  /// This implements decorator pattern. You can build many
  /// different solvers decorating tabu_search class in different
  /// ways.
  /// 
  template<typename move_manager_type>
  class tabu_search : public abstract_search<move_manager_type>
  {
  public:
    typedef tabu_search<move_manager_type> search_type;
    /// @brief Creates a tabu Search instance.
    ///
    /// @param starting_solution  The working solution (this
    /// will be modified during search).
    ///
    /// @param best_recorder A solution recorder used to record the
    /// best solution found during the search.
    ///
    /// @param move_manager_inst A problem specific implementation of the
    /// move_manager_type used to generate the neighborhood.
    ///
    /// @param tabus The tabu list used to decorate this search
    /// instance.
    ///
    /// @param aspiration The aspiration criteria to use in this tabu
    /// search.
    ///
    /// @param termination The termination criteria used to terminate the
    /// search process, this is an extension to the standard Simulated
    /// Annealing: you can give a termination criteria that termiantes
    /// when temperature reaches 0.
    ///
    tabu_search(feasible_solution& starting_solution, 
		solution_recorder& best_recorder, 
		move_manager_type& move_manager_inst,
		tabu_list_chain& tabus,
		aspiration_criteria_chain& aspiration,
		termination_criteria_chain& termination);

    tabu_search(const search_type&);
    search_type& operator=(const search_type&);

    virtual 
    ~tabu_search() {}

    /// @brief This method starts the tabu search process.
    /// 
    /// Remember that this is a minimization process.
    ///
    /// An exception mets::no_moves_error is risen when no move
    /// is possible.
    void 
    search() 
      throw(no_moves_error);
    
    enum {
      ASPIRATION_CRITERIA_MET = abstract_search<move_manager_type>::LAST,
      LAST
    };

    /// @brief The tabu list used by this tabu search
    const tabu_list_chain& 
    get_tabu_list() const { return tabu_list_m; }
    
    /// @brief The aspiration criteria used by this tabu search
    const aspiration_criteria_chain& 
    get_aspiration_criteria() const { return aspiration_criteria_m; }

    /// @brief The termination criteria used by this tabu search
    const termination_criteria_chain& 
    get_termination_criteria() const { return termination_criteria_m; }
  protected:
    tabu_list_chain& tabu_list_m;
    aspiration_criteria_chain& aspiration_criteria_m;
    termination_criteria_chain& termination_criteria_m;
  };

  /// @brief Simplistic implementation of a tabu-list.
  ///
  /// This class implements one of the simplest and less
  /// memory hungry tabu lists. This tabu list memorizes
  /// only the moves (not the solutions).
  ///
  /// Moves must be of mets::mana_move type.
  ///
  /// The comparison between moves is demanded to the
  /// move implementation.
  ///
  /// A mets::mana_move is tabu if it's in the tabu list by means 
  /// of its operator== and hash function.
  class simple_tabu_list 
    : public tabu_list_chain
  {
  public:
    /// @brief Ctor. Makes a tabu list of the specified tenure.
    ///
    /// @param tenure Tenure (length) of the tabu list
    simple_tabu_list(unsigned int tenure) 
      : tabu_list_chain(tenure), 
	tabu_moves_m(), 
	tabu_hash_m(tenure) {}

    /// @brief Ctor. Makes a tabu list of the specified tenure.
    ///
    /// @param tenure Tenure (length) of the tabu list
    /// @param next Next list to invoke when this returns false
    simple_tabu_list(tabu_list_chain* next, unsigned int tenure) 
      : tabu_list_chain(next, tenure), 
	tabu_moves_m(), 
	tabu_hash_m(tenure) {}

    /// @brief Destructor
    ~simple_tabu_list();

    /// @brief Make move a tabu.
    /// 
    /// This implementation simply remembers "tenure" moves.
    ///
    /// @param sol The current working solution
    /// @param mov The move to make tabu
    void
    tabu(feasible_solution& sol, /* const */ move& mov);

    /// @brief True if the move is tabu for the given solution.
    ///
    /// This implementation considers tabu each move already made less
    /// then tenure() moves ago.
    ///
    /// @param sol The current working solution
    /// @param mov The move to make tabu
    /// @return True if this move was already made during the last
    /// tenure iterations
    bool
    is_tabu(feasible_solution& sol, move& mov) const;

  protected:
    typedef std::deque<move*> move_list_type;
#if defined (METSLIB_TR1_BOOST)
    typedef boost::unordered_map<
          mana_move*, // Key type
          int, //insert a move and the number of times it's present in the list
          mana_move_hash,
          dereferenced_equal_to<mana_move*> > move_map_type;
#elif defined (METSLIB_HAVE_UNORDERED_MAP) && !defined (METSLIB_TR1_MIXED_NAMESPACE)
    typedef std::unordered_map<
      mana_move*, // Key type
      int, //insert a move and the number of times it's present in the list
      mana_move_hash, 
      dereferenced_equal_to<mana_move*> > move_map_type;
#else
    typedef std::tr1::unordered_map<
      mana_move*, // Key type
      int, //insert a move and the number of times it's present in the list
      mana_move_hash, 
      dereferenced_equal_to<mana_move*> > move_map_type;
#endif    
    move_list_type tabu_moves_m;
    move_map_type tabu_hash_m;
  };

  /// @brief Aspiration criteria implementation.
  ///
  /// This is one of the best known aspiration criteria
  /// ready to be used in your tabu-search implementation.
  ///
  /// This aspiration criteria is met when a tabu move would result in
  /// a global improvement.
  class best_ever_criteria : public aspiration_criteria_chain
  {
  public:
    explicit
    best_ever_criteria(double min_improvement = 1e-6);

    explicit
    best_ever_criteria(aspiration_criteria_chain* next,
		       double min_improvement = 1e-6);
    
    void 
    reset();

    void
    accept(feasible_solution& fs, move& mov, gol_type evaluation);

    bool 
    operator()(feasible_solution& fs, move& mov, gol_type evaluation) const;

  protected:
    gol_type best_m;
    gol_type tolerance_m;
  };
  
  /// @}
}

template<typename move_manager_t>
mets::tabu_search<move_manager_t>::
tabu_search (feasible_solution& starting_solution, 
	     solution_recorder& best_recorder, 
	     move_manager_t& move_manager_inst,
	     tabu_list_chain& tabus,
	     aspiration_criteria_chain& aspiration,
	     termination_criteria_chain& termination)
  : abstract_search<move_manager_t>(starting_solution, 
				    best_recorder, 
				    move_manager_inst),
    tabu_list_m(tabus),
    aspiration_criteria_m(aspiration),
    termination_criteria_m(termination)
{}

template<typename move_manager_t>
void mets::tabu_search<move_manager_t>::search()
  throw(no_moves_error)
{
  typedef abstract_search<move_manager_t> base_t;
  while(!termination_criteria_m(base_t::working_solution_m))
    {
      // call listeners
      base_t::step_m = base_t::ITERATION_BEGIN;
      this->notify();

      base_t::moves_m.refresh(base_t::working_solution_m);
      
      typename move_manager_t::iterator best_movit = base_t::moves_m.end(); 
      gol_type best_move_cost = std::numeric_limits<gol_type>::max();
      
      for(typename move_manager_t::iterator movit = base_t::moves_m.begin(); 
	  movit != base_t::moves_m.end(); ++movit)
	{
	  // evaluate proposed move
	  gol_type cost = (*movit)->evaluate(base_t::working_solution_m);
	  
	  // save tabu status
	  bool is_tabu = tabu_list_m.is_tabu(base_t::working_solution_m, 
						 **movit);

	  // for each non-tabu move record the best one
	  if(cost < best_move_cost)
	    {
	      
	      bool aspiration_criteria_met = false;
	      
	      // not interesting if this is not a tabu move (and if we
	      // are not improving over other moves)
	      if(is_tabu) 
		{
		  aspiration_criteria_met = 
		    aspiration_criteria_m(base_t::working_solution_m, 
					  **movit,
					  cost);
		}
	      
	      if(!is_tabu || aspiration_criteria_met)
		{
		  best_move_cost = cost;
		  best_movit = base_t::current_move_m = movit;
		  if(aspiration_criteria_met)
		    {
		      base_t::step_m = ASPIRATION_CRITERIA_MET;
		      this->notify();
		    }
		}
	    }
	} // end for each move
      
      if(best_movit == base_t::moves_m.end())
	throw no_moves_error();

      // make move tabu
      tabu_list_m.tabu(base_t::working_solution_m, **best_movit);

      // do the best non tabu move (unless overridden by aspiration
      // criteria, of course)
      (*best_movit)->apply(base_t::working_solution_m);

      // call listeners
      base_t::step_m = base_t::MOVE_MADE;
      this->notify();
      
      aspiration_criteria_m.accept(base_t::working_solution_m, 
				   **best_movit, 
				   best_move_cost);
      
      if(base_t::solution_recorder_m.accept(base_t::working_solution_m))
	{
	  base_t::step_m = base_t::IMPROVEMENT_MADE;
	  this->notify();
	}

      // call listeners
      base_t::step_m = base_t::ITERATION_END;
      this->notify();
      
    } // end while(!termination)
}

// chain of responsibility

inline void
mets::tabu_list_chain::tabu(feasible_solution& sol, /* const */ move& mov)
{
  if(next_m)
    next_m->tabu(sol, mov);
}

inline bool
mets::tabu_list_chain::is_tabu(feasible_solution& sol, /* const */ move& mov) const
{
  if(next_m) 
    return next_m->is_tabu(sol, mov);
  else 
    return false;
}

inline mets::simple_tabu_list::~simple_tabu_list()
{ 
  for(move_map_type::iterator m = tabu_hash_m.begin(); 
      m!=tabu_hash_m.end(); ++m)
    delete m->first;
}

inline void
mets::simple_tabu_list::tabu(feasible_solution& sol, /* const */ move& mov)
{
  mana_move* mc = 
    dynamic_cast<mana_move&>(mov).opposite_of();

  // This does nothing if the move was already tabu (can happen when
  // aspiration criteria is met).
  std::pair<move_map_type::iterator, bool> 
    insert_result = tabu_hash_m.insert(std::make_pair(mc, 1));

  // If it was already in the map, increase the counter
  if(!insert_result.second) 
    {
      insert_result.first->second++;
      delete mc;
      mc = insert_result.first->first;
    }
  // Always add the move at the end of the list (when aspiration
  // criteria is met a move can be present more than one time in this
  // list: this is correct, so the last made move is always the last
  // in the queue). 
  tabu_moves_m.push_back(mc);

  // Since we use the hash size, the tenure is the number of different
  // moves on the list (the tabu_moves_m can have more than tenure
  // elements)
  while(tabu_hash_m.size() > this->tenure())
    {
      // update hash map *and* list structures
      move_map_type::iterator elem = tabu_hash_m.find
	(dynamic_cast<mana_move*>(tabu_moves_m.front()));
      elem->second--;
      if(elem->second == 0) 
	{
	  mana_move* tmp = elem->first;
	  tabu_hash_m.erase(elem);
	  delete tmp;
	}
      tabu_moves_m.pop_front();
    }
  tabu_list_chain::tabu(sol, mov);
}

inline bool
mets::simple_tabu_list::is_tabu(feasible_solution& sol, move& mov) const
{
  // hash set. very fast but requires C++ ISO TR1 extension
  // and an hash function in every move (Omega(1)).
  bool tabu = (tabu_hash_m.find(&dynamic_cast<mana_move&>(mov)) 
	       != tabu_hash_m.end());

  if(tabu)
    return true;

  return tabu_list_chain::is_tabu(sol, mov);
}

//////////////////////////////////////////////////////////////////////////
// aspiration_criteria_chain
inline void 
mets::aspiration_criteria_chain::reset()
{
  if(next_m)
    return next_m->reset();
}

inline void 
mets::aspiration_criteria_chain::accept(feasible_solution& fs, 
					move& mov,
					gol_type eval)
{
  if(next_m) next_m->accept(fs, mov, eval);
}

inline bool 
mets::aspiration_criteria_chain::operator()(feasible_solution& fs, 
					    move& mov,
					    gol_type eval) const
{
  if(next_m)
    return next_m->operator()(fs, mov, eval);
  else
    return false;
}

//////////////////////////////////////////////////////////////////////////
// best_ever_criteria
inline mets::best_ever_criteria::best_ever_criteria(double tolerance) 
  : aspiration_criteria_chain(),
    best_m(std::numeric_limits<gol_type>::max()),
    tolerance_m(tolerance)
{ }

inline mets::best_ever_criteria::best_ever_criteria(aspiration_criteria_chain* next, double tolerance) 
  : aspiration_criteria_chain(next),
    best_m(std::numeric_limits<gol_type>::max()),
    tolerance_m(tolerance)
{ }
    
inline void 
mets::best_ever_criteria::reset()
{
  best_m = std::numeric_limits<mets::gol_type>::max();
  aspiration_criteria_chain::reset();
}

inline void
mets::best_ever_criteria::accept(feasible_solution& fs, 
				 move& mov, 
				 gol_type eval) 
{
  best_m = std::min(dynamic_cast<const evaluable_solution&>(fs).cost_function(), best_m);
  aspiration_criteria_chain::accept(fs, mov, eval);
}  

inline bool 
mets::best_ever_criteria::operator()(feasible_solution& fs, 
				     move& mov, 
				     gol_type eval) const
{ 
  /// the solution is the solution before applying mov.
  if(eval < best_m - tolerance_m)
    return true;
  else
    return aspiration_criteria_chain::operator()(fs, mov, eval); 
}

#endif
