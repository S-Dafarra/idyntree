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

#ifndef IDYNTREE_OPTIMALCONTROL_MULTIPLESHOOTINGSOLVER_H
#define IDYNTREE_OPTIMALCONTROL_MULTIPLESHOOTINGSOLVER_H

#include "iDynTree/OptimalControlSolver.h"
#include "iDynTree/OptimizationProblem.h"
#include "iDynTree/Core/SparseMatrix.h"

#include <memory>

namespace iDynTree {

    class VectorDynSize;

    namespace optimalcontrol {

        class OptimalControlProblem;

        namespace integrators {
            class Integrator;
        }
        using namespace integrators;

        /**
         * @warning This class is still in active development, and so API interface can change between iDynTree versions.
         * \ingroup iDynTreeExperimental
         */

        class MultipleShootingTranscription : public OptimizationProblem {

            friend class MultipleShootingSolver;

            MultipleShootingTranscription();

            MultipleShootingTranscription(const std::shared_ptr<OptimalControlProblem> problem, const std::shared_ptr<Integrator> integrationMethod);

            MultipleShootingTranscription(const MultipleShootingTranscription& other) = delete;

            size_t setControlMeshPoints();

            bool setMeshPoints();

            bool setOptimalControlProblem(const std::shared_ptr<OptimalControlProblem> problem);

            bool setIntegrator(const std::shared_ptr<Integrator> integrationMethod);

            bool setStepSizeBounds(const double minStepSize, const double maxStepsize);

            bool setControlPeriod(double period);

            bool setAdditionalStateMeshPoints(const std::vector<double>& stateMeshes);

            bool setAdditionalControlMeshPoints(const std::vector<double>& controlMeshes);

            class MultipleShootingTranscriptionPimpl;
            MultipleShootingTranscriptionPimpl *m_pimpl;

        public:

            virtual ~MultipleShootingTranscription() override;

            virtual bool prepare() override;

            virtual void reset() override;

            virtual bool getInfo(unsigned int& numberOfVariables, unsigned int& numberOfConstraints,
                                 unsigned int& numberOfNonZerosConstraintsJacobian, unsigned int& numberOfNonZerosHessian) override;

//            bool getBoundsInfo(VectorDynSize& variablesLowerBounds, VectorDynSize& variableUpperBounds,
//                               VectorDynSize& constraintsLowerBounds, VectorDynSize& constraintsUpperBounds,
//                               double infinity = 1e19) override;

//            bool evaluateCostFunction(const VectorDynSize& variables, double& costValue) override;

//            bool evaluateCostGradient(const VectorDynSize& variables, VectorDynSize& gradient) override;

//            bool evaluateConstraints(const VectorDynSize& variables, VectorDynSize& constraints) override;

//            bool evaluateConstraintsJacobian(const VectorDynSize& variables, SparseMatrix& jacobian) override;

//            virtual bool evaluateHessian(const VectorDynSize& variables, double costMultiplier,
//                                         const VectorDynSize& constraintsMultipliers, SparseMatrix<RowMajor>& hessian) override;
        };


        class MultipleShootingSolver : public OptimalControlSolver {

        public:
            MultipleShootingSolver(const std::shared_ptr<OptimalControlProblem>& ocProblem);

            MultipleShootingSolver(const MultipleShootingSolver& other) = delete;

            bool setStepSizeBounds(double minStepSize, double maxStepsize);

            bool setIntegrator(const std::shared_ptr<Integrator> integrationMethod);

            bool setControlPeriod(double period);

            bool setAdditionalStateMeshPoints(const std::vector<double>& stateMeshes);

            bool setAdditionalControlMeshPoints(const std::vector<double>& controlMeshes);

            // FIXME: These two cannot be used as VectorDynTree
            // as they are trajectories, not single vectors
            void setInitialGuess(const iDynTree::VectorDynSize& initialGuess);
            const iDynTree::VectorDynSize& lastSolution();


            virtual bool initialize() override;
            virtual bool solve() override;

        private:

            class MultipleShootingSolverPimpl;
            MultipleShootingSolverPimpl* m_pimpl;

            std::shared_ptr<MultipleShootingTranscription> m_transcription;
        };

    }
}

#endif /* end of include guard: IDYNTREE_OPTIMALCONTROL_MULTIPLESHOOTINGSOLVER_H */
