// METSlib source file - model.hh                                -*- C++ -*-
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

#ifndef METS_MODEL_HH_
#define METS_MODEL_HH_

namespace mets {

  /// @brief Type of the objective/cost function.
  ///
  /// You should be able to change this to "int" for your uses
  /// to improve performance if it suffice, no guarantee.
  ///
  typedef double gol_type;

  /// @brief Exception risen when some algorithm has no more moves to
  /// make.
  class no_moves_error 
    : public std::runtime_error
  {
  public:
    no_moves_error() 
      : std::runtime_error("There are no more available moves.") {}
    no_moves_error(const std::string message) 
      : std::runtime_error(message) {}
  };

  /// @brief A sequence function object useful as an STL generator.
  ///
  /// Returns start, start+1, ... 
  ///
  class sequence
  {
  public:
    /// @brief A sequence class starting at start.
    sequence(int start = 0) 
      : value_m(start) 
    { }
    /// @brief Subscript operator. Each call increments the value of
    /// the sequence.
    int operator()() 
    { return value_m++; }
  protected:
    int value_m;
  };

  /// @brief An interface for prototype objects.
  class clonable {
  public:
    virtual 
    ~clonable() {};
    virtual clonable* 
    clone() const = 0;
  };

  /// @brief An interface for hashable objects.
  class hashable {
  public:
    virtual 
    ~hashable() {};
    virtual size_t 
    hash() const = 0;
  };

  /// @brief An interface for copyable objects.
  class copyable {
  public:
    virtual 
    ~copyable() {};
    virtual void 
    copy_from(const copyable&) = 0;
  };

  /// @brief An interface for printable objects.
  class printable {
  public:
    virtual 
    ~printable() {}
    virtual void 
    print(std::ostream& /*os*/) const { };
  };

  /// @defgroup model Model
  /// @{

  /// @brief interface of a feasible solution space to be searched
  /// with tabu search.
  ///
  /// Note that "feasible" is not intended w.r.t. the constraint of
  /// the problem but only regarding the space we want the local
  /// search to explore. From time to time allowing solutions to
  /// explore unfeasible regions is non only allowed, but encouraged
  /// to improve tabu search performances. In those cases the
  /// objective function should probably account for unfeasibility
  /// with a penalty term.
  ///
  /// This is the most generic solution type and is useful only if you
  /// implement your own solution recorder and
  /// max-noimprove. Otherwise you might want to derive from an
  /// evaluable_solution or from a permutation_problem class,
  /// depending on your problem type.
  ///
  class feasible_solution
  {
  public:
    /// @brief Virtual dtor.
    virtual
    ~feasible_solution() 
    { }

  };


  /// @brief A copyable and evaluable solution implementation,
  /// 
  /// All you need, if you implement your own mets::solution_recorder,
  /// is to derive from the almost empty
  /// mets::feasible_solution. However, if you want to use the
  /// provided mets::best_ever_recorder you need to derive from this
  /// class (that also defines an interface to copy and evaluate a
  /// solution).
  ///
  /// @see mets::best_ever_recorder
  class evaluable_solution : public feasible_solution, 
			     public copyable
  { 
  public:
    /// @brief Cost function to be minimized.
    ///
    /// The cost function is the target that the search algorithm
    /// tries to minimize.
    ///
    /// You must implement this for your problem.
    ///
    virtual gol_type 
    cost_function() const = 0;
  };

  /// @brief An abstract permutation problem.
  ///
  /// The permutation problem provides a skeleton to rapidly prototype
  /// permutation problems (such as Assignment problem, Quadratic AP,
  /// TSP, and so on). The skeleton holds a pi_m variable that, at
  /// each step, contains a permutation of the numbers in [0..n-1].
  ///
  /// A few operators are provided to randomly choose a solution, to
  /// perturbate a solution with some random swaps and to simply swap
  /// two items in the list.
  ///
  /// @see mets::swap_elements
  class permutation_problem: public evaluable_solution 
  {
  public:
    
    /// @brief Unimplemented.
    permutation_problem(); 

    /// @brief Inizialize pi_m = {0, 1, 2, ..., n-1}.
    permutation_problem(int n) : pi_m(n), cost_m(0.0)
    { std::generate(pi_m.begin(), pi_m.end(), sequence(0)); }

    /// @brief Copy from another permutation problem, if you introduce
    /// new member variables remember to override this and to call
    /// permutation_problem::copy_from in the overriding code.
    ///
    /// @param other the problem to copy from
    void copy_from(const copyable& other);

    /// @brief: Compute cost of the whole solution.
    ///
    /// You will need to override this one.
    virtual gol_type
    compute_cost() const = 0;

    /// @brief: Evaluate a swap.
    ///
    /// Implement this method to evaluate the change in the cost
    /// function after the swap (without actually modifying the
    /// solution). The method should return the difference in cost
    /// between the current position and the position after the swap
    /// (negative if decreasing and positive otherwise).
    ///
    /// To obtain maximal performance from the algorithm it is
    /// essential, whenever possible, to only compute the cost update
    /// and not the whole cost function.
    virtual gol_type
    evaluate_swap(int i, int j) const = 0;

    /// @brief The size of the problem.
    /// Do not override unless you know what you are doing.
    size_t 
    size() const
    { return pi_m.size(); }

    /// @brief Returns the cost of the current solution. The default
    /// implementation provided returns the protected
    /// mets::permutation_problem::cost_m member variable. Do not
    /// override unless you know what you are doing.
    gol_type cost_function() const 
    { return cost_m; }

    /// @brief Updates the cost with the one computed by the subclass.
    /// Do not override unless you know what you are doing.
    void
    update_cost() 
    { cost_m = compute_cost(); }
    
    /// @brief: Apply a swap and update the cost.
    /// Do not override unless you know what you are doing.
    void
    apply_swap(int i, int j)
    { cost_m += evaluate_swap(i,j); std::swap(pi_m[i], pi_m[j]); }
    

  protected:
    std::vector<int> pi_m;
    gol_type cost_m;
    template<typename random_generator> 
    friend void random_shuffle(permutation_problem& p, random_generator& rng);
  };


  /// @brief Shuffle a permutation problem (generates a random starting point).
  ///
  /// @see mets::permutation_problem
  template<typename random_generator>
  void random_shuffle(permutation_problem& p, random_generator& rng)
  {
#if defined (METSLIB_TR1_BOOST)
    boost::uniform_int<size_t> unigen;
    boost::variate_generator<random_generator&,
      boost::uniform_int<size_t> >gen(rng, unigen);
#elif defined (METSLIB_HAVE_UNORDERED_MAP) && !defined (METSLIB_TR1_MIXED_NAMESPACE)
    std::uniform_int<size_t> unigen;
    std::variate_generator<random_generator&, 
      std::uniform_int<size_t> >gen(rng, unigen);
#else
    std::tr1::uniform_int<size_t> unigen;
    std::tr1::variate_generator<random_generator&, 
      std::tr1::uniform_int<size_t> >gen(rng, unigen);
#endif
    std::random_shuffle(p.pi_m.begin(), p.pi_m.end(), gen);
    p.update_cost();
  }
  
  /// @brief Perturbate a problem with n swap moves.
  ///
  /// @see mets::permutation_problem
  template<typename random_generator>
  void perturbate(permutation_problem& p, unsigned int n, random_generator& rng)
  {
#if defined (METSLIB_TR1_BOOST)
    boost::uniform_int<> int_range;
#elif defined (METSLIB_HAVE_UNORDERED_MAP) && !defined (METSLIB_TR1_MIXED_NAMESPACE)
    std::uniform_int<> int_range;
#else
    std::tr1::uniform_int<> int_range;
#endif
    for(unsigned int ii=0; ii!=n;++ii) 
      {
	int p1 = int_range(rng, p.size());
	int p2 = int_range(rng, p.size());
	while(p1 == p2) 
	  p2 = int_range(rng, p.size());
	p.apply_swap(p1, p2);
      }
  }
    
  /// @brief Move to be operated on a feasible solution.
  ///
  /// You must implement this (one or more types are allowed) for your
  /// problem.
  ///
  /// You must provide an apply as well as an evaluate method.
  ///
  /// NOTE: this interface changed from 0.4.x to 0.5.x. The change was
  /// needed to provide a more general interface.
  class move
  {
  public:

    virtual 
    ~move() 
    { }; 

    ///
    /// @brief Evaluate the cost after the move.
    ///
    /// What if we make this move? Local searches can be speed up by a
    /// substantial amount if we are able to efficiently evaluate the
    /// cost of the neighboring solutions without actually changing
    /// the solution.
    virtual gol_type
    evaluate(const feasible_solution& sol) const = 0;

    ///
    /// @brief Operates this move on sol.
    ///
    /// This should actually change the solution.
    virtual void
    apply(feasible_solution& sol) const = 0;


  };

  /// @brief A Mana Move is a move that can be automatically made tabu
  /// by the mets::simple_tabu_list.
  ///
  /// If you implement this class you can use the
  /// mets::simple_tabu_list as a ready to use tabu list.
  /// 
  /// You must implement a clone() method, provide an hash funciton
  /// and provide a operator==() method that is responsible to find if
  /// a move is equal to another.
  ///
  /// NOTE: If the desired behaviour is to declare tabu the *opposite*
  /// of the last made move you can achieve that behavioud override
  /// the opposite_of() method as well.
  ///
  class mana_move : 
    public move, 
    public clonable, 
    public hashable
  {
  public:
    
    /// @brief Create and return a new move that is the reverse of
    /// this one
    ///
    /// By default this just calls "clone". If this method is not
    /// overridden the mets::simple_tabu_list declares tabu the last
    /// made move. Reimplementing this method it is possibile to
    /// actually declare as tabu the opposite of the last made move
    /// (if we moved a to b we can declare tabu moving b to a).
    virtual mana_move*
    opposite_of() const 
    { return static_cast<mana_move*>(clone()); }
    
    /// @brief Tell if this move equals another w.r.t. the tabu list
    /// management (for mets::simple_tabu_list)
    virtual bool 
    operator==(const mana_move& other) const = 0;
    
  };

  template<typename rndgen> class swap_neighborhood; // fw decl

  /// @brief A mets::mana_move that swaps two elements in a
  /// mets::permutation_problem.
  ///
  /// Each instance swaps two specific objects.
  ///
  /// @see mets::permutation_problem, mets::mana_move
  ///
  class swap_elements : public mets::mana_move 
  {
  public:  

    /// @brief A move that swaps from and to.
    swap_elements(int from, int to) 
      : p1(std::min(from,to)), p2(std::max(from,to)) 
    { }
    
    /// @brief Virtual method that applies the move on a point
    gol_type
    evaluate(const mets::feasible_solution& s) const
    { const permutation_problem& sol = 
	static_cast<const permutation_problem&>(s);
      return sol.cost_function() + sol.evaluate_swap(p1, p2); }
    
    /// @brief Virtual method that applies the move on a point
    void
    apply(mets::feasible_solution& s) const
    { permutation_problem& sol = static_cast<permutation_problem&>(s);
      sol.apply_swap(p1, p2); }
            
    /// @brief Clones this move (so that the tabu list can store it)
    clonable* 
    clone() const
    { return new swap_elements(p1, p2); }

    /// @brief An hash function used by the tabu list (the hash value is
    /// used to insert the move in an hash set).
    size_t
    hash() const
    { return (p1)<<16^(p2); }
    
    /// @brief Comparison operator used to tell if this move is equal to
    /// a move in the simple tabu list move set.
    bool 
    operator==(const mets::mana_move& o) const;
    
    /// @brief Modify this swap move.
    void change(int from, int to)
    { p1 = std::min(from,to); p2 = std::max(from,to); }

  protected:
    int p1; ///< the first element to swap
    int p2; ///< the second element to swap

    template <typename> 
    friend class swap_neighborhood;
  };

  /// @brief A mets::mana_move that swaps a subsequence of elements in
  /// a mets::permutation_problem.
  ///
  /// @see mets::permutation_problem, mets::mana_move
  ///
  class invert_subsequence : public mets::mana_move 
  {
  public:  

    /// @brief A move that swaps from and to.
    invert_subsequence(int from, int to) 
      : p1(from), p2(to) 
    { }
    
    /// @brief Virtual method that applies the move on a point
    gol_type
    evaluate(const mets::feasible_solution& s) const;

    /// @brief Virtual method that applies the move on a point
    void
    apply(mets::feasible_solution& s) const;
        
    clonable* 
    clone() const
    { return new invert_subsequence(p1, p2); }

    /// @brief An hash function used by the tabu list (the hash value is
    /// used to insert the move in an hash set).
    size_t
    hash() const
    { return (p1)<<16^(p2); }
    
    /// @brief Comparison operator used to tell if this move is equal to
    /// a move in the tabu list.
    bool 
    operator==(const mets::mana_move& o) const;
    
    void change(int from, int to)
    { p1 = from; p2 = to; }
    
  protected:
    int p1; ///< the first element to swap
    int p2; ///< the second element to swap
    
    // template <typename> 
    // friend class invert_full_neighborhood;
  };

  /// @brief A neighborhood generator.
  ///
  /// This is a sample implementation of the neighborhood exploration
  /// concept. You can still derive from this class and implement the
  /// refresh method, but, since version 0.5.x you don't need to.
  ///
  /// To implement your own move manager you should simply adhere to
  /// the following concept:
  ///
  /// provide an iterator, and size_type types, a begin() and end()
  /// method returning iterators to a move collection. The switch to a
  /// template based move_manager was made so that you can use any
  /// iterator type that you want. This allows, between other things,
  /// to implement intelligent iterators that dynamically return
  /// moves.
  ///
  /// The move manager can represent both Variable and Constant
  /// Neighborhoods.
  ///
  /// To make a constant neighborhood put moves in the moves_m queue
  /// in the constructor and implement an empty <code>void
  /// refresh(feasible_solution&)</code> method.
  ///
  class move_manager
  {
  public:
    ///
    /// @brief Initialize the move manager with an empty list of moves
    move_manager() 
      : moves_m() 
    { }

    /// @brief Virtual destructor
    virtual ~move_manager() 
    { }

    /// @brief Selects a different set of moves at each iteration.
    virtual void 
    refresh(mets::feasible_solution& s) = 0;
    
    /// @brief Iterator type to iterate over moves of the neighborhood
    typedef std::deque<move*>::iterator iterator;
    
    /// @brief Size type
    typedef std::deque<move*>::size_type size_type;

    /// @brief Begin iterator of available moves queue.
    iterator begin() 
    { return moves_m.begin(); }

    /// @brief End iterator of available moves queue.
    iterator end() 
    { return moves_m.end(); }

    /// @brief Size of the neighborhood.
    size_type size() const 
    { return moves_m.size(); }

  protected:
    std::deque<move*> moves_m; ///< The moves queue
    move_manager(const move_manager&);
  };
  

  /// @brief Generates a stochastic subset of the neighborhood.
#if defined (METSLIB_TR1_BOOST)
  template<typename random_generator = boost::minstd_rand0>
#elif defined (METSLIB_HAVE_UNORDERED_MAP) && !defined (METSLIB_TR1_MIXED_NAMESPACE)
  template<typename random_generator = std::minstd_rand0>
#else
  template<typename random_generator = std::tr1::minstd_rand0>
#endif
  class swap_neighborhood : public mets::move_manager
  {
  public:
    /// @brief A neighborhood exploration strategy for mets::swap_elements.
    ///
    /// This strategy selects *moves* random swaps 
    ///
    /// @param r a random number generator (e.g. an instance of
    /// std::tr1::minstd_rand0 or std::tr1::mt19936)
    ///
    /// @param moves the number of swaps to add to the exploration
    ///
    swap_neighborhood(random_generator& r, 
		      unsigned int moves);

    /// @brief Dtor.
    ~swap_neighborhood();

    /// @brief Selects a different set of moves at each iteration.
    void refresh(mets::feasible_solution& s);
    
  protected:
    random_generator& rng;

#if defined (METSLIB_TR1_BOOST)
    boost::uniform_int<> int_range;
#elif defined (METSLIB_HAVE_UNORDERED_MAP) && !defined (METSLIB_TR1_MIXED_NAMESPACE)
    std::uniform_int<> int_range;
#else
    std::tr1::uniform_int<> int_range;
#endif
    unsigned int n;

    void randomize_move(swap_elements& m, unsigned int size);
  };

  //________________________________________________________________________
  template<typename random_generator>
  mets::swap_neighborhood< random_generator
			   >::swap_neighborhood(random_generator& r, 
						unsigned int moves)
			     : mets::move_manager(), rng(r), int_range(0), n(moves)
  { 
    // n simple moves
    for(unsigned int ii = 0; ii != n; ++ii) 
      moves_m.push_back(new swap_elements(0,0));
  }  
  
  template<typename random_generator>
  mets::swap_neighborhood<random_generator>::~swap_neighborhood()
  {
    // delete all moves
    for(iterator ii = begin(); ii != end(); ++ii)
      delete (*ii);
  }

  template<typename random_generator>
  void
  mets::swap_neighborhood<random_generator>::refresh(mets::feasible_solution& s)
  {
    permutation_problem& sol = dynamic_cast<permutation_problem&>(s);
    iterator ii = begin();
    
    // the first n are simple qap_moveS
    for(unsigned int cnt = 0; cnt != n; ++cnt)
      {
	swap_elements* m = static_cast<swap_elements*>(*ii);
	randomize_move(*m, sol.size());
	++ii;
      }
    
  }
  
  template<typename random_generator>
  void
  mets::swap_neighborhood<random_generator
			  >::randomize_move(swap_elements& m, unsigned int size)
  {
    int p1 = int_range(rng, size);
    int p2 = int_range(rng, size);
    while(p1 == p2) 
      p2 = int_range(rng, size);
    // we are friend, so we know how to handle the nuts&bolts of
    // swap_elements
    m.p1 = std::min(p1,p2); 
    m.p2 = std::max(p1,p2); 
  }

  /// @brief Generates a the full swap neighborhood.
  class swap_full_neighborhood : public mets::move_manager
  {
  public:
    /// @brief A neighborhood exploration strategy for mets::swap_elements.
    ///
    /// This strategy selects *moves* random swaps.
    ///
    /// @param size the size of the problem
    swap_full_neighborhood(int size) : move_manager()
    {
      for(int ii(0); ii!=size-1; ++ii)
	for(int jj(ii+1); jj!=size; ++jj)
	  moves_m.push_back(new swap_elements(ii,jj));
    } 

    /// @brief Dtor.
    ~swap_full_neighborhood() { 
      for(move_manager::iterator it = moves_m.begin(); 
	  it != moves_m.end(); ++it)
	delete *it;
    }
    
    /// @brief Use the same set set of moves at each iteration.
    void refresh(mets::feasible_solution& /*s*/) { }
    
  };


  /// @brief Generates a the full subsequence inversion neighborhood.
  class invert_full_neighborhood : public mets::move_manager
  {
  public:
    invert_full_neighborhood(int size) : move_manager()
    {
      for(int ii(0); ii!=size; ++ii)
	for(int jj(0); jj!=size; ++jj)
	  if(ii != jj)
	    moves_m.push_back(new invert_subsequence(ii,jj));
    } 

    /// @brief Dtor.
    ~invert_full_neighborhood() { 
      for(std::deque<move*>::iterator it = moves_m.begin(); 
	  it != moves_m.end(); ++it)
	delete *it;
    }

    /// @brief This is a static neighborhood
    void 
    refresh(mets::feasible_solution& /*s*/)
    { }

  };

  /// @}

  /// @brief Functor class to allow hash_set of moves (used by tabu list)
  class mana_move_hash 
  {
  public:
    size_t operator()(mana_move const* mov) const 
    {return mov->hash();}
  };
  
  /// @brief Functor class to allow hash_set of moves (used by tabu list)
  template<typename Tp>
  struct dereferenced_equal_to 
  {
    bool operator()(const Tp l, 
		    const Tp r) const 
    { return l->operator==(*r); }
  };

}

//________________________________________________________________________
inline void
mets::permutation_problem::copy_from(const mets::copyable& other)
{
  const mets::permutation_problem& o = 
    dynamic_cast<const mets::permutation_problem&>(other);
  pi_m = o.pi_m;
  cost_m = o.cost_m;
}

//________________________________________________________________________
inline bool
mets::swap_elements::operator==(const mets::mana_move& o) const
{
  try {
    const mets::swap_elements& other = 
      dynamic_cast<const mets::swap_elements&>(o);
    return (this->p1 == other.p1 && this->p2 == other.p2);
  } catch (std::bad_cast& e) {
    return false;
  }
}

//________________________________________________________________________

inline void
mets::invert_subsequence::apply(mets::feasible_solution& s) const
{ 
  mets::permutation_problem& sol = 
    static_cast<mets::permutation_problem&>(s);
  int size = static_cast<int>(sol.size());
  int top = p1 < p2 ? (p2-p1+1) : (size+p2-p1+1);
  for(int ii(0); ii!=top/2; ++ii)
    {
      int from = (p1+ii)%size;
      int to = (size+p2-ii)%size;
      assert(from >= 0 && from < size);
      assert(to >= 0 && to < size);
      sol.apply_swap(from, to); 
    }
}

inline mets::gol_type
mets::invert_subsequence::evaluate(const mets::feasible_solution& s) const
{ 
  const mets::permutation_problem& sol = 
    static_cast<const mets::permutation_problem&>(s);
  int size = static_cast<int>(sol.size());
  int top = p1 < p2 ? (p2-p1+1) : (size+p2-p1+1);
  mets::gol_type eval = 0.0;
  for(int ii(0); ii!=top/2; ++ii)
    {
      int from = (p1+ii)%size;
      int to = (size+p2-ii)%size;
      assert(from >= 0 && from < size);
      assert(to >= 0 && to < size);
      eval += sol.evaluate_swap(from, to); 
    }
  return eval;
}

inline bool
mets::invert_subsequence::operator==(const mets::mana_move& o) const
{
  try {
    const mets::invert_subsequence& other = 
      dynamic_cast<const mets::invert_subsequence&>(o);
    return (this->p1 == other.p1 && this->p2 == other.p2);
  } catch (std::bad_cast& e) {
    return false;
  }
}

#endif
