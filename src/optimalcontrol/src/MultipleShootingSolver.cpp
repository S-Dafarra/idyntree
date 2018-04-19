/*
 * Copyright (C) 2014,2017 Fondazione Istituto Italiano di Tecnologia
 * Authors: Francesco Romano
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 * Originally developed for Prioritized Optimal Control (2014)
 * Refactored in 2017.
 * Design inspired by
 * - ACADO toolbox (http://acado.github.io)
 * - ADRL Control Toolbox (https://adrlab.bitbucket.io/ct/ct_doc/doc/html/index.html)
 */

#include "iDynTree/OCSolvers/MultipleShootingSolver.h"

#include "iDynTree/OptimalControlProblem.h"
#include "iDynTree/DynamicalSystem.h"
#include "iDynTree/Integrator.h"
#include <iDynTree/TimeRange.h>

#include <iDynTree/Core/VectorDynSize.h>

#include <vector>
#include <cassert>
#include <algorithm>

// List of TODOs
// TODO: varying time interval
// TODO: decide later how to structure the hierarchy
// TODO: understand if some parameters should go in the Integrator class or not

namespace iDynTree {
    namespace optimalcontrol
    {
        class MultipleShootingTranscription::MultipleShootingTranscriptionPimpl{
        public:
            std::shared_ptr<OptimalControlProblem> ocproblem;
            std::shared_ptr<Integrator> integrator;
            unsigned int variables, constraints, nonZerosJacobian, nonZerosHessian;
            bool prepared;
            std::vector<double> userDefinedMeshPoints;
            std::vector<double> inputMeshPoints;
            std::vector<double> meshPointsTimes;
            double minStepSize, maxStepSize;


            MultipleShootingTranscriptionPimpl()
            : ocproblem(nullptr)
            ,integrator(nullptr)
            ,variables(0)
            ,constraints(0)
            ,nonZerosJacobian(0)
            ,nonZerosHessian(0)
            ,prepared(false)
            ,minStepSize(0.001)
            ,maxStepSize(0.01)
            {}

            MultipleShootingTranscriptionPimpl(const std::shared_ptr<OptimalControlProblem> problem,
                                            const std::shared_ptr<Integrator> integrationMethod)
            :ocproblem(problem)
            ,integrator(integrationMethod)
            ,variables(0)
            ,constraints(0)
            ,nonZerosJacobian(0)
            ,nonZerosHessian(0)
            ,prepared(false)
            ,minStepSize(0.001)
            ,maxStepSize(0.01)
            {}
        };

        MultipleShootingTranscription::MultipleShootingTranscription()
        :m_pimpl(new MultipleShootingTranscriptionPimpl())
        {
            assert(m_pimpl);
        }

        MultipleShootingTranscription::MultipleShootingTranscription(const std::shared_ptr<OptimalControlProblem> problem,
                                                               const std::shared_ptr<Integrator> integrationMethod)
        :m_pimpl(new MultipleShootingTranscriptionPimpl(problem, integrationMethod))
        {
            assert(m_pimpl);
        }

        MultipleShootingTranscription::~MultipleShootingTranscription()
        {
            if (m_pimpl){
                delete m_pimpl;
                m_pimpl = nullptr;
            }
        }

        bool MultipleShootingTranscription::setOptimalControlProblem(const std::shared_ptr<OptimalControlProblem> problem)
        {
            if (m_pimpl->ocproblem){
                reportError("OCMultipleShootingInstance", "setOptimalControlProblem", "The OptimalControlProblem for this instance has already been set.");
                return false;
            }

            m_pimpl->ocproblem = problem;
            return true;
        }

        bool MultipleShootingTranscription::setIntegrator(const std::shared_ptr<Integrator> integrationMethod)
        {
            if (m_pimpl->integrator){
                reportError("OCMultipleShootingInstance", "setIntegrator", "The integration method for this instance has already been set.");
                return false;
            }

            m_pimpl->integrator = integrationMethod;
            return true;
        }

        bool MultipleShootingTranscription::setStepSizeBounds(const double minStepSize, const double maxStepsize)
        {
            if (minStepSize <= 0){
                reportError("OCMultipleShootingInstance", "setStepSizeBounds","The minimum step size is expected to be positive.");
                return false;
            }

            if (minStepSize > maxStepsize){
                reportError("OCMultipleShootingInstance", "setStepSizeBounds","The maximum step size is expected to be greater than the minimum.");
                return false;
            }

            m_pimpl->minStepSize = minStepSize;
            m_pimpl->maxStepSize = maxStepsize;

            return true;
        }

        bool MultipleShootingTranscription::prepare()
        {
            if ((m_pimpl->ocproblem->finalTime() - m_pimpl->ocproblem->initialTime()) < m_pimpl->minStepSize){
                reportError("OCMultipleShootingInstance", "prepare",
                            "The time horizon defined in the OptimalControlProblem is smaller than the minimum step size.");
                return false;
            }

            if (!m_pimpl->prepared){
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
            }

            if (!(m_pimpl->integrator->setMaximumStepSize(m_pimpl->maxStepSize))){
                reportError("OCMultipleShootingInstance", "prepare","Error while setting the maximum step size to the integrator.");
                return false;
            }

            //HERE I HAVE TO DECIDE THE TIME ASSOCIATED WITH THE OPTIMIZATION VARIABLES.
            //First put a mesh point on the initial point, the final point, the initial_point+dTmax, the user defined points, and in all the init and end time of all costs and costraints.

            std::vector<TimeRange> &constraintsTRs = m_pimpl->ocproblem->getConstraintsTimeRanges(), &costsTRs = m_pimpl->ocproblem->getCostsTimeRanges();

            size_t constraintsMeshPoints = constraintsTRs.size() * 2;
            size_t costsMeshPoints = costsTRs.size() * 2;
            size_t additionalMeshPoints = m_pimpl->userDefinedMeshPoints.size() + 3; //The three are the first instant, the last instant, and one at t0+dTmax
            size_t inputMeshPointSize = additionalMeshPoints + constraintsMeshPoints + costsMeshPoints;

            if (m_pimpl->inputMeshPoints.size() < inputMeshPointSize)
                m_pimpl->inputMeshPoints.resize(inputMeshPointSize);

            m_pimpl->inputMeshPoints[0] = m_pimpl->ocproblem->initialTime();
            m_pimpl->inputMeshPoints[1] = m_pimpl->ocproblem->finalTime();
            m_pimpl->inputMeshPoints[2] = m_pimpl->ocproblem->initialTime() + m_pimpl->maxStepSize;

            size_t offset = 3;

            for (size_t i = 0; i < m_pimpl->userDefinedMeshPoints.size(); ++i)
                m_pimpl->inputMeshPoints[i + offset] = m_pimpl->userDefinedMeshPoints[i];

            offset = additionalMeshPoints;

            for (size_t i = 0; i < constraintsTRs.size(); ++i){
                m_pimpl->inputMeshPoints[2*i + offset] = constraintsTRs[i].initTime();
                m_pimpl->inputMeshPoints[2*i + 1 + offset] = constraintsTRs[i].endTime();
            }

            offset += constraintsMeshPoints;

            for (size_t i = 0; i < constraintsTRs.size(); ++i){
                m_pimpl->inputMeshPoints[2*i + 3] = constraintsTRs[i].initTime();
                m_pimpl->inputMeshPoints[2*i + 1 + 3] = constraintsTRs[i].endTime();
            }

            std::sort(m_pimpl->inputMeshPoints.begin(), m_pimpl->inputMeshPoints.end());

            for (auto time : m_pimpl->inputMeshPoints){
                if (time < m_pimpl->ocproblem->initialTime())
                    time = m_pimpl->ocproblem->finalTime() + m_pimpl->maxStepSize;

    //check also of the points greater than the maximum and if the difference is too short
            }


            m_pimpl->prepared = true;
            return true;
        }

        void MultipleShootingTranscription::reset()
        {
            m_pimpl->prepared = false;
            m_pimpl->variables = 0;
            m_pimpl->constraints = 0;
        }

        bool MultipleShootingTranscription::getInfo(unsigned int &numberOfVariables, unsigned int &numberOfConstraints,
                                                 unsigned int &numberOfNonZerosConstraintsJacobian, unsigned int &numberOfNonZerosHessian)
        {

        }

        bool MultipleShootingTranscription::evaluateHessian(const VectorDynSize &variables, double costMultiplier, const VectorDynSize &constraintsMultipliers, SparseMatrix<RowMajor> &hessian)
        {

        }


        // MARK: Private implementation
        class MultipleShootingSolver::MultipleShootingSolverPimpl {
        public:
            MultipleShootingSolverPimpl(const std::shared_ptr<OptimalControlProblem> problem)
            : controlProblem(problem) {}

            std::shared_ptr<OptimalControlProblem> controlProblem;
            iDynTree::VectorDynSize lastSolution;

            iDynTree::VectorDynSize optimisationVariable;
        };


        // MARK: Class implementation

        MultipleShootingSolver::MultipleShootingSolver(const std::shared_ptr<OptimalControlProblem> &ocProblem)
        : OptimalControlSolver(ocProblem)
        , m_pimpl(new MultipleShootingSolverPimpl(ocProblem))
        , m_transcription(std::make_shared<MultipleShootingTranscription>())
        {
            assert(m_pimpl);
            m_transcription->setOptimalControlProblem(ocProblem);
        }

        bool MultipleShootingSolver::setStepSizeBounds(const double minStepSize, const double maxStepsize)
        {
            return m_transcription->setStepSizeBounds(minStepSize, maxStepsize);
        }

        bool MultipleShootingSolver::setIntegrator(const std::shared_ptr<Integrator> integrationMethod)
        {
            return m_transcription->setIntegrator(integrationMethod);
        }

        void MultipleShootingSolver::setInitialGuess(const iDynTree::VectorDynSize& initialGuess)
        {
            assert(m_pimpl);
            m_pimpl->lastSolution = initialGuess;
        }

        const iDynTree::VectorDynSize& MultipleShootingSolver::lastSolution()
        {
            assert(m_pimpl);
            return m_pimpl->lastSolution;
        }

        bool MultipleShootingSolver::initialize()
        {
            assert(m_pimpl);

            return true;
        }

        bool MultipleShootingSolver::solve()
        {
            assert(m_pimpl);
            return false;
        }
    }
}
