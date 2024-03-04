// (C) Copyright International Business Machines (IBM) 2005, 2007
// All Rights Reserved.
// This code is published under the Eclipse Public License.
//
// Authors :
// Pierre Bonami, IBM
//
// Date : 26/09/2006

#ifndef IpoptSolver_HPP
#define IpoptSolver_HPP
#include "BonTNLPSolver.hpp"
#include "IpIpoptApplication.hpp"


namespace Bonmin
{
  class IpoptSolver: public TNLPSolver
  {
  public:
  class UnsolvedIpoptError: public TNLPSolver::UnsolvedError
    {
    public:
      UnsolvedIpoptError(int errorNum,
          Ipopt::SmartPtr<TMINLP2TNLP> problem,
          std::string name):
          TNLPSolver::UnsolvedError(errorNum, problem, name)
      {}
      virtual const std::string& errorName() const;

      virtual const std::string& solverName() const;
      virtual ~UnsolvedIpoptError()
      {}
    private:
      static std::string errorNames [17];
      static std::string solverName_;
    };

    virtual UnsolvedError * newUnsolvedError(int num,
        Ipopt::SmartPtr<TMINLP2TNLP> problem,
        std::string name)
    {
      return new UnsolvedIpoptError(num, problem, name);
    }



    /// Constructor
    IpoptSolver(bool createEmpty = false);

/// Constructor with Passed in journalist, registered options, options
    IpoptSolver(Ipopt::SmartPtr<Bonmin::RegisteredOptions> roptions,
        Ipopt::SmartPtr<Ipopt::OptionsList> options,
        Ipopt::SmartPtr<Ipopt::Journalist> journalist,
        const std::string & prefix);

/// Constructor with Passed in journalist, registered options, options
    IpoptSolver(Ipopt::SmartPtr<Bonmin::RegisteredOptions> roptions,
        Ipopt::SmartPtr<Ipopt::OptionsList> options,
        Ipopt::SmartPtr<Ipopt::Journalist> journalist);

    /// Copy constructor
    IpoptSolver(const IpoptSolver &other);

    ///virtual copy constructor
    virtual Ipopt::SmartPtr<TNLPSolver> clone();

    /// Virtual destructor
    virtual ~IpoptSolver();

    /** Initialize the TNLPSolver (read options from params_file)
    */
    virtual bool Initialize(std::string params_file);

    /** Initialize the TNLPSolver (read options from istream is)
    */
    virtual bool Initialize(std::istream& is);

    /** @name Solve methods */
    //@{
    /// Solves a problem expresses as a TNLP
    virtual TNLPSolver::ReturnStatus OptimizeTNLP(const Ipopt::SmartPtr<Ipopt::TNLP> & tnlp);

    /// Resolves a problem expresses as a TNLP
    virtual TNLPSolver::ReturnStatus ReOptimizeTNLP(const Ipopt::SmartPtr<Ipopt::TNLP> & tnlp);

    /// Set the warm start in the solver
    virtual bool setWarmStart(const CoinWarmStart * warm,
        Ipopt::SmartPtr<TMINLP2TNLP> tnlp);

   /// Get warm start used in last optimization
   virtual CoinWarmStart * getUsedWarmStart(Ipopt::SmartPtr<TMINLP2TNLP> tnlp) const;


    /// Get the warm start form the solver
    virtual CoinWarmStart * getWarmStart(Ipopt::SmartPtr<Bonmin::TMINLP2TNLP> tnlp) const;

    virtual CoinWarmStart * getEmptyWarmStart() const;

    /** Check that warm start object is valid.*/
    virtual bool warmStartIsValid(const CoinWarmStart * ws) const;

    /// Enable the warm start options in the solver
    virtual void enableWarmStart();

    /// Disable the warm start options in the solver
    virtual void disableWarmStart();

    //@}

    /// Get the CpuTime of the last optimization.
    virtual double CPUTime();

    /// Get the iteration count of the last optimization.
    virtual int IterationCount();

    /// turn off all output from the solver
    virtual void setOutputToDefault();
    /// turn on all output from the solver
    virtual void forceSolverOutput(int log_level);

    /// Get the solver name
    virtual std::string & solverName()
    {
      return solverName_;
    }

    /// Register this solver options into passed roptions
    static void RegisterOptions(Ipopt::SmartPtr<Bonmin::RegisteredOptions> roptions)
    {
      Ipopt::IpoptApplication::RegisterAllIpoptOptions(GetRawPtr(roptions));
    }



    /// Return status of last optimization
    Ipopt::ApplicationReturnStatus getOptStatus() const
    {
      return optimizationStatus_;
    }

    Ipopt::IpoptApplication& getIpoptApp()
    {
      return *app_;
    }

    virtual int errorCode() const
    {
      return (int) optimizationStatus_;
    }
  private:
    /** Set default Ipopt parameters for use in a MINLP */
    void setMinlpDefaults(Ipopt::SmartPtr< Ipopt::OptionsList> Options);

    /** get Bonmin return status from Ipopt one. */
    TNLPSolver::ReturnStatus solverReturnStatus(Ipopt::ApplicationReturnStatus optimization_status) const;

    /** Ipopt application */
    Ipopt::SmartPtr<Ipopt::IpoptApplication> app_;
    /** return status of last optimization.*/
    Ipopt::ApplicationReturnStatus optimizationStatus_;
    //@}


    /** Flag to indicate if last problem solved had 0 dimension. (in this case Ipopt was not called).*/
    bool problemHadZeroDimension_;

    /** Warm start strategy :
    <ol>
    <li> no warm start,</li>
    <li> simple warm start (optimal point),</li>
    <li> more elaborate strategies (interior point...).</li>
    </ol>
    */
    int warmStartStrategy_;

    /** flag remembering if we want to use warm start option */
    bool enable_warm_start_;

    /** flag remembering if we have call the Optimize method of the
        IpoptInterface before */
    bool optimized_before_;
    //name of solver (Ipopt)
    static std::string  solverName_;
  };
}
#endif

