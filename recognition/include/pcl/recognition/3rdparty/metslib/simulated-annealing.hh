// METSlib source file - simulated-annealing.hh                  -*- C++ -*-
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

#ifndef METS_SIMULATED_ANNEALING_HH_
#define METS_SIMULATED_ANNEALING_HH_

namespace mets {

  /// @defgroup simulated_annealing Simulated Annealing
  /// @{

  /// @brief Cooling criteria (for Simulated Annealing).
  ///
  /// @see mets::simulated_annealing
  ///
  /// An abstract annealing schedule. Implementations
  /// should decide the new temperature every time the 
  /// subscript operator is called (every search iteration)
  class abstract_cooling_schedule
  {
  public:
    /// @brief Constructor
    abstract_cooling_schedule() 
    { }

    /// @brief Virtual destructor
    virtual
    ~abstract_cooling_schedule() 
    { }

    /// @brief The function that updates the SA temperature.
    ///
    /// @param temp The actual annealing temperature.
    /// @param fs The current working solution.
    /// @return The new scheduled temperature.
    virtual double
    operator()(double temp, feasible_solution& fs) = 0;
  };

  /// @brief Search by Simulated Annealing.
  template<typename move_manager_type>
  class simulated_annealing : public mets::abstract_search<move_manager_type>
  {
  public:
    using search_type = simulated_annealing<move_manager_type>;
    /// @brief Creates a search by simulated annealing instance.
    ///
    /// @param working The working solution (this will be modified
    /// during search).
    ///
    /// @param recorder A solution recorder (possibly holding a
    /// different solution instance) used to record the best solution
    /// found.
    ///
    /// @param moveman A problem specific implementation of the
    /// move_manager_type used to generate the neighborhood (the
    /// choice of the neighbourhood and its exploration greatly
    /// influences the algorithm quality and speed).
    ///
    /// @param tc The termination criteria used to terminate the
    /// search process, this is an extension to the standard Simulated
    /// Annealing: the algorithm terminates either when the
    /// termination criterion is met or when the temperature is <= 0.
    ///
    /// @param cs The annealing schedule that will be used by the
    /// algorithm to regulate the temperature at each iteration (many
    /// have been proposed in literature and influence the quality and
    /// speed of the algorithm).
    ///
    /// @param starting_temp The starting SA temperature (this is an
    /// important parameter that depends on the problem and will
    /// influence the search quality and duration).
    ///
    /// @param K The "Boltzmann" constant that we want ot use (default is 1).
    simulated_annealing(evaluable_solution& starting_point,
			solution_recorder& recorder,
			move_manager_type& moveman,
			termination_criteria_chain& tc,
			abstract_cooling_schedule& cs,
			double starting_temp,
			double stop_temp = 1e-7,
			double K = 1.0);
    
    /// purposely not implemented (see Effective C++)
    simulated_annealing(const simulated_annealing&);
    simulated_annealing& operator=(const simulated_annealing&);
    
    /// @brief This method starts the simulated annealing search
    /// process.
    ///
    /// Remember that this is a minimization process.
    ///
    virtual void
    search();

    void setApplyAndEvaluate(bool b) {
      apply_and_evaluate = b;
    }

    /// @brief The current annealing temperature.
    ///
    /// @return The current temperature of the algorithm.
    double 
    current_temp() const 
    { return current_temp_m; }

    /// @brief The annealing schedule instance.
    ///
    /// @return The cooling schedule used by this search process.
    const abstract_cooling_schedule& 
    cooling_schedule() const
    { return cooling_schedule_m; }

  protected:
    termination_criteria_chain& termination_criteria_m;
    abstract_cooling_schedule& cooling_schedule_m;
    double starting_temp_m;
    double stop_temp_m;
    double current_temp_m;
    double K_m;

#if defined (METSLIB_TR1_BOOST)
    boost::uniform_real<double> ureal;
    boost::mt19937 rng;
    boost::variate_generator< boost::mt19937, boost::uniform_real<double> > gen;
#elif defined (METSLIB_HAVE_UNORDERED_MAP) && !defined (METSLIB_TR1_MIXED_NAMESPACE)
    std::uniform_real<double> ureal;
    std::mt19937 rng;
    std::variate_generator< std::mt19937, std::uniform_real<double> > gen;
#else
    std::tr1::uniform_real<double> ureal;
    std::tr1::mt19937 rng;
    std::tr1::variate_generator< std::tr1::mt19937, std::tr1::uniform_real<double> > gen;
#endif

    bool apply_and_evaluate;
  };
    
  /// @brief Original ECS proposed by Kirkpatrick
  class exponential_cooling
    : public abstract_cooling_schedule
  {
  public:
    exponential_cooling(double alpha = 0.95)
      : abstract_cooling_schedule(), factor_m(alpha) 
    { if(alpha >= 1) throw std::runtime_error("alpha must be < 1"); }
    double
    operator()(double temp, feasible_solution& /*fs*/)
    { return temp*factor_m; }
  protected:
    double factor_m;
  };

  /// @brief Alternative LCS proposed by Randelman and Grest
  class linear_cooling
    : public abstract_cooling_schedule
  {
  public:
    linear_cooling(double delta = 0.1)
      : abstract_cooling_schedule(), decrement_m(delta)
    { if(delta <= 0) throw std::runtime_error("delta must be > 0"); }
    double
    operator()(double temp, feasible_solution& /*fs*/)
    { return std::max(0.0, temp-decrement_m); }
  protected:
    double decrement_m;
  };

  /// @}
}

template<typename move_manager_t>
mets::simulated_annealing<move_manager_t>::
simulated_annealing(evaluable_solution& working,
		    solution_recorder& recorder,
		    move_manager_t& moveman,
		    termination_criteria_chain& tc,
		    abstract_cooling_schedule& cs,
		    double starting_temp, 
		    double stop_temp,
		    double K)
  : abstract_search<move_manager_t>(working, recorder, moveman),
    termination_criteria_m(tc), cooling_schedule_m(cs),
    starting_temp_m(starting_temp), stop_temp_m(stop_temp),
    current_temp_m(), K_m(K),
    ureal(0.0,1.0), rng(), gen(rng, ureal), apply_and_evaluate(false)
{ 
}

template<typename move_manager_t>
void
mets::simulated_annealing<move_manager_t>::search()
{
  using base_t = abstract_search<move_manager_t>;

  current_temp_m = starting_temp_m;
  while(!termination_criteria_m(base_t::working_solution_m) 
        && current_temp_m > stop_temp_m)
    {
      gol_type actual_cost = 
	static_cast<mets::evaluable_solution&>(base_t::working_solution_m)
	.cost_function();
      /*gol_type best_cost =
	static_cast<mets::evaluable_solution&>(base_t::working_solution_m)
	.cost_function();*/

      base_t::moves_m.refresh(base_t::working_solution_m);
      for(typename move_manager_t::iterator movit = base_t::moves_m.begin(); 
          movit != base_t::moves_m.end(); ++movit)
      {
        // apply move and record proposed cost function
        gol_type cost;
        if(apply_and_evaluate) {
          cost = (*movit)->apply_and_evaluate(base_t::working_solution_m);
        } else {
          cost = (*movit)->evaluate(base_t::working_solution_m);
        }

        double delta = (static_cast<double>(cost-actual_cost));
        if(delta < 0 || gen() < exp(-delta/(K_m*current_temp_m)))
        {
          // accepted: apply, record, exit for and lower temperature
          if(!apply_and_evaluate)
            (*movit)->apply(base_t::working_solution_m);

          base_t::current_move_m = movit;
          if(base_t::solution_recorder_m.accept(base_t::working_solution_m))
          {
            base_t::step_m = base_t::IMPROVEMENT_MADE;
            this->notify();
          }

          base_t::step_m = base_t::MOVE_MADE;
          this->notify();
          break;
        }
        else if(apply_and_evaluate)
        {
          (*movit)->unapply(base_t::working_solution_m);
        }
      } // end for each move
      
      current_temp_m = 
      cooling_schedule_m(current_temp_m, base_t::working_solution_m);
    }
}
#endif
