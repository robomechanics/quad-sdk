// (C) Copyright International Business Machines (IBM) 2006, 2007
// All Rights Reserved.
// This code is published under the Eclipse Public License.
//
// Authors :
// Pierre Bonami, IBM
//
// Date : 26/09/2006


#ifndef TNLPSolver_H
#define TNLPSolver_H
#include "IpTNLP.hpp"
#include "BonTMINLP2TNLP.hpp"

//Some declarations
#include "IpOptionsList.hpp"
#include "CoinWarmStart.hpp"
#include "BonRegisteredOptions.hpp"
#include "CoinTime.hpp"
namespace Bonmin  {
/** This is a generic class for calling an NLP solver to solve a TNLP.
    A TNLPSolver is able to solve and resolve a problem, it has some options (stored
    with Ipopt OptionList structure and registeredOptions) it produces some statistics (in SolveStatisctics and sometimes some errorCodes.
*/
class TNLPSolver: public Ipopt::ReferencedObject{
 public:

  enum ReturnStatus /** Standard return statuses for a solver*/{
    iterationLimit = -3/** Solver reached iteration limit. */,
    timeLimit = 5/** Solver reached iteration limit. */,
    doesNotConverge = -8/** Algorithm does not converge.*/,
    computationError = -2/** Some error was made in the computations. */,
    notEnoughFreedom = -1/** not enough degrees of freedom.*/,
    illDefinedProblem = -4/** The solver finds that the problem is not well defined. */,
    illegalOption =-5/** An option is not valid. */,
    externalException =-6/** Some unrecovered exception occurred in an external tool used by the solver. */,
    exception =-7/** Some unrocevered exception */,
    solvedOptimal = 1/** Problem solved to an optimal solution.*/,
    solvedOptimalTol =2/** Problem solved to "acceptable level of tolerance. */,
    provenInfeasible =3/** Infeasibility Proven. */,
    unbounded = 4/** Problem is unbounded.*/,
    numReturnCodes/**Fake member to know size*/
  };



//#############################################################################

  /** We will throw this error when a problem is not solved.
      Eventually store the error code from solver*/
  class UnsolvedError
  {
  public:
    /** Constructor */
    UnsolvedError(int errorNum = -10000, 
                  Ipopt::SmartPtr<TMINLP2TNLP> model = NULL,
                  std::string name="")
    :
     errorNum_(errorNum),
     model_(model),
     name_(name)
    {if(name_=="") 
{
#ifndef NDEBUG
	std::cerr<<"FIXME"<<std::endl;
#endif
}}
    /** Print error message.*/
    void printError(std::ostream & os);
    /** Get the string corresponding to error.*/
    virtual const std::string& errorName() const = 0;
    /** Return the name of the solver. */
    virtual const std::string& solverName() const = 0;
    /** Return error number. */
    int errorNum() const{
    return errorNum_;}
    /** destructor. */
    virtual ~UnsolvedError(){}
    /** write files with differences between input model and
        this one */
    void writeDiffFiles(const std::string prefix=std::string()) const;
  private:
    /** Error code (solver dependent). */
    int errorNum_;

    /** model_ on which error occured*/
    Ipopt::SmartPtr< TMINLP2TNLP > model_;

    /** name of the model on which error occured. */
    std::string name_;
  }
  ;

  virtual UnsolvedError * newUnsolvedError(int num,
					   Ipopt::SmartPtr<TMINLP2TNLP> problem,
					   std::string name) = 0;
 


  /// default Constructor
   TNLPSolver();

  ///Constructor with options initialization
TNLPSolver(Ipopt::SmartPtr<Bonmin::RegisteredOptions> roptions,
           Ipopt::SmartPtr<Ipopt::OptionsList> options,
           Ipopt::SmartPtr<Ipopt::Journalist> journalist,
           const std::string & prefix);

  ///virtual copy constructor
  virtual Ipopt::SmartPtr<TNLPSolver> clone() = 0;

   /// Virtual destructor
   virtual ~TNLPSolver();

   /** Initialize the TNLPSolver (read options from params_file)
   */
   virtual bool Initialize(std::string params_file) = 0;

   /** Initialize the TNLPSolver (read options from istream is)
   */
   virtual bool Initialize(std::istream& is) = 0;

   /** @name Solve methods */
   //@{
   /// Solves a problem expresses as a TNLP 
   virtual ReturnStatus OptimizeTNLP(const Ipopt::SmartPtr<Ipopt::TNLP> & tnlp) = 0;

   /// Resolves a problem expresses as a TNLP 
   virtual ReturnStatus ReOptimizeTNLP(const Ipopt::SmartPtr<Ipopt::TNLP> & tnlp) = 0;

  /// Set the warm start in the solver
  virtual bool setWarmStart(const CoinWarmStart * warm, 
                            Ipopt::SmartPtr<TMINLP2TNLP> tnlp) = 0;

/// Get warm start used in last optimization
  virtual CoinWarmStart * getUsedWarmStart(Ipopt::SmartPtr<TMINLP2TNLP> tnlp) const = 0;

  /// Get the warm start form the solver
  virtual CoinWarmStart * getWarmStart(Ipopt::SmartPtr<TMINLP2TNLP> tnlp) const = 0;

  virtual CoinWarmStart * getEmptyWarmStart() const = 0;

  /** Check that warm start object is valid.*/
  virtual bool warmStartIsValid(const CoinWarmStart * ws) const = 0;  

  /// Enable the warm start options in the solver
  virtual void enableWarmStart() = 0;

  /// Disable the warm start options in the solver
  virtual void disableWarmStart() = 0;
   //@}

  ///Get a pointer to a journalist
  Ipopt::SmartPtr<Ipopt::Journalist> journalist(){
    return journalist_;}

   ///Get a pointer to RegisteredOptions (generally used to add new ones)
   Ipopt::SmartPtr<Bonmin::RegisteredOptions> roptions(){
     return roptions_;}

   /// Get the options (for getting their values).
   Ipopt::SmartPtr<const Ipopt::OptionsList> options() const {
     return ConstPtr(options_);}

   /// Get the options (for getting and setting their values).
   Ipopt::SmartPtr<Ipopt::OptionsList> options() {
     return options_;}

  /// Get the prefix
  const char * prefix(){
    return prefix_.c_str();
  }
   /// Register this solver options into passed roptions
static void RegisterOptions(Ipopt::SmartPtr<Bonmin::RegisteredOptions> roptions){}

   /// Get the CpuTime of the last optimization.
   virtual double CPUTime() = 0;

   /// Get the iteration count of the last optimization.
   virtual int IterationCount() = 0;


  /// turn off all output from the solver 
  virtual void setOutputToDefault() = 0 ;
  /// turn on all output from the solver
  virtual void forceSolverOutput(int log_level) = 0;
  /// Get the solver name
  virtual std::string & solverName() = 0;

    /** Say if an optimization status for a problem which failed is recoverable
        (problem may be solvable).*/
  bool isRecoverable(ReturnStatus &r);

  /** Setup for a global time limit for solver.*/
  void setup_global_time_limit(double time_limit){
    time_limit_ = time_limit + 5;
    start_time_ = CoinCpuTime();
  }

  /** Say if return status is an error.*/
  bool isError(ReturnStatus &r){
    return r < 0;}
  /** Error code (solver specific).*/
virtual int errorCode() const = 0;
protected:
   /** Determine if problem is of dimension zero and if it is check if solution
       is feasible.*/
   bool zeroDimension(const Ipopt::SmartPtr<Ipopt::TNLP> &tnlp, 
		     ReturnStatus &optimization_status);

   /** Initializes options and journalist.*/
   void initializeOptionsAndJournalist();

    /** Storage of Journalist for output */
    Ipopt::SmartPtr<Ipopt::Journalist> journalist_;
    
    /** List of Options */
    Ipopt::SmartPtr<Ipopt::OptionsList> options_;
    
    /** Registered Options */
    Ipopt::SmartPtr<Bonmin::RegisteredOptions> roptions_;
   
    /** Prefix to use for reading bonmin's options.*/
   std::string prefix_;
   /** Global start time.*/
   double start_time_;

   /** Global time limit.*/
   double time_limit_;

   /** To record default log level.*/
   int default_log_level_;
  /// Copy Constructor
  TNLPSolver(const TNLPSolver & other);

};
}
#endif


