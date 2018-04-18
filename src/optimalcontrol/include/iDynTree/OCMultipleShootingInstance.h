/*
 * Copyright (C) 2014,2017 Fondazione Istituto Italiano di Tecnologia
 * Authors: Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 * Originally developed for Prioritized Optimal Control (2014)
 * Refactored in 2017.
 * Design inspired by
 * - ACADO toolbox (http://acado.github.io)
 * - ADRL Control Toolbox (https://adrlab.bitbucket.io/ct/ct_doc/doc/html/index.html)
 */

#ifndef IDYNTREE_OPTIMALCONTROL_OCMULTIPLESHOOTINGINSTANCE_H
#define IDYNTREE_OPTIMALCONTROL_OCMULTIPLESHOOTINGINSTANCE_H

#include "iDynTree/OptimizationInstance.h"
#include "iDynTree/Core/SparseMatrix.h"
#include <memory>

namespace iDynTree {

    class VectorDynSize;

    namespace optimalcontrol {

        namespace integrators{
            class Integrator;
        }
        using namespace integrators;

        class OptimalControlProblem;

        /**
         * @warning This class is still in active development, and so API interface can change between iDynTree versions.
         * \ingroup iDynTreeExperimental
         */
        class OCMultipleShootingInstance : public OptimizationInstance {
        public:
            OCMultipleShootingInstance();

            OCMultipleShootingInstance(const std::shared_ptr<OptimalControlProblem> problem, const std::shared_ptr<Integrator> integrationMethod);

            OCMultipleShootingInstance(const OCMultipleShootingInstance& other) = delete;

            virtual ~OCMultipleShootingInstance() override;

            bool setOptimalControlProblem(const std::shared_ptr<OptimalControlProblem> problem);

            bool setIntegrator(const std::shared_ptr<Integrator> integrationMethod);

            bool setStepSizeBounds(const double minStepSize, const double maxStepsize);

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

            virtual bool evaluateHessian(const VectorDynSize& variables, double costMultiplier,
                                         const VectorDynSize& constraintsMultipliers, SparseMatrix<RowMajor>& hessian) override;

        private:
            class OCMultipleShootingInstancePimpl;
            OCMultipleShootingInstancePimpl *m_pimpl;
        };

    }
}

#endif // IDYNTREE_OPTIMALCONTROL_OCMULTIPLESHOOTINGINSTANCE_H
