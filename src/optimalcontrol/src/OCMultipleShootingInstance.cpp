/*
 * Copyright (C) 2014,2017 Fondazione Istituto Italiano di Tecnologia
 * Authors: Francesco Romano, Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 * Originally developed for Prioritized Optimal Control (2014)
 * Refactored in 2017.
 * Design inspired by
 * - ACADO toolbox (http://acado.github.io)
 * - ADRL Control Toolbox (https://adrlab.bitbucket.io/ct/ct_doc/doc/html/index.html)
 */

#include <iDynTree/OCMultipleShootingInstance.h>
#include <iDynTree/Integrator.h>
#include <iDynTree/OptimalControlProblem.h>

#include <iDynTree/Core/VectorDynSize.h>

#include <vector>
#include <cassert>

namespace iDynTree{
    namespace optimalcontrol {

        using namespace integrators;

        class OCMultipleShootingInstance::OCMultipleShootingInstancePimpl{
        public:
            std::shared_ptr<OptimalControlProblem> ocproblem;
            std::shared_ptr<Integrator> integrator;
            unsigned int variables, constraints, nonZerosJacobian, nonZerosHessian;
            bool prepared;
            std::vector<double> meshPoints;


            OCMultipleShootingInstancePimpl()
            : ocproblem(nullptr)
            ,integrator(nullptr)
            ,variables(0)
            ,constraints(0)
            ,nonZerosJacobian(0)
            ,nonZerosHessian(0)
            ,prepared(false)
            {}

            OCMultipleShootingInstancePimpl(const std::shared_ptr<OptimalControlProblem> problem,
                                            const std::shared_ptr<Integrator> integrationMethod)
            :ocproblem(problem)
            ,integrator(integrationMethod)
            ,variables(0)
            ,constraints(0)
            ,nonZerosJacobian(0)
            ,nonZerosHessian(0)
            ,prepared(false)
            {}
        };

        OCMultipleShootingInstance::OCMultipleShootingInstance()
        :m_pimpl(new OCMultipleShootingInstancePimpl())
        {
            assert(m_pimpl);
        }

        OCMultipleShootingInstance::OCMultipleShootingInstance(const std::shared_ptr<OptimalControlProblem> problem,
                                                               const std::shared_ptr<Integrator> integrationMethod)
        :m_pimpl(new OCMultipleShootingInstancePimpl(problem, integrationMethod))
        {
            assert(m_pimpl);
        }

        OCMultipleShootingInstance::~OCMultipleShootingInstance()
        {
            if (m_pimpl){
                delete m_pimpl;
                m_pimpl = nullptr;
            }
        }

        bool OCMultipleShootingInstance::setOptimalControlProblem(const std::shared_ptr<OptimalControlProblem> problem)
        {
            if (m_pimpl->ocproblem){
                reportError("OCMultipleShootingInstance", "setOptimalControlProblem", "The OptimalControlProblem for this instance has already been set.");
                return false;
            }

            m_pimpl->ocproblem = problem;
            return true;
        }

        bool OCMultipleShootingInstance::setIntegrator(const std::shared_ptr<Integrator> integrationMethod)
        {
            if (m_pimpl->integrator){
                reportError("OCMultipleShootingInstance", "setIntegrator", "The integration method for this instance has already been set.");
                return false;
            }

            m_pimpl->integrator = integrationMethod;
            return true;
        }

        bool OCMultipleShootingInstance::prepare()
        {
            if (m_pimpl->ocproblem->dynamicalSystem().expired()){
                if (m_pimpl->integrator->dynamicalSystem().expired()){
                    reportError("OCMultipleShootingInstance", "prepare",
                                "No dynamical system set, neither to the OptimalControlProblem nor to the Integrator object.");
                    return false;
                }
                if (!(m_pimpl->ocproblem->setDynamicalSystemConstraint(m_pimpl->integrator->dynamicalSystem().lock()))){
                    reportError("OCMultipleShootingInstance", "prepare",
                                "Error while setting the dynamicalSystem to the OptimalControlProblem using the one pointer provided by the Integrator object.");
                    return false;
                }
            } else {
                if (!(m_pimpl->integrator->dynamicalSystem().expired()) &&
                        (m_pimpl->integrator->dynamicalSystem().lock() != m_pimpl->ocproblem->dynamicalSystem().lock())){
                    reportError("OCMultipleShootingInstance", "prepare",
                                "The selected OptimalControlProblem and the Integrator point to two different dynamical systems.");
                    return false;
                }
                if (!(m_pimpl->integrator->setDynamicalSystem(m_pimpl->ocproblem->dynamicalSystem().lock()))){
                    reportError("OCMultipleShootingInstance", "prepare",
                                "Error while setting the dynamicalSystem to the Integrator using the one pointer provided by the OptimalControlProblem object.");
                    return false;
                }
            }
            //HERE I HAVE TO DECIDE THE NUMBER OF OPTIMIZATION VARIABLES.
            m_pimpl->prepared = true;
            return true;
        }

        bool OCMultipleShootingInstance::getInfo(unsigned int &numberOfVariables, unsigned int &numberOfConstraints,
                                                 unsigned int &numberOfNonZerosConstraintsJacobian, unsigned int &numberOfNonZerosHessian)
        {

        }

        bool OCMultipleShootingInstance::evaluateHessian(const VectorDynSize &variables, double costMultiplier, const VectorDynSize &constraintsMultipliers, SparseMatrix<RowMajor> &hessian)
        {

        }




    }
}
