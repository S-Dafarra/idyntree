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

#ifndef IDYNTREE_OPTIMALCONTROL_OPTIMIZATIONINSTANCE_H
#define IDYNTREE_OPTIMALCONTROL_OPTIMIZATIONINSTANCE_H

#include <iDynTree/Core/SparseMatrix.h>

namespace iDynTree {

    class VectorDynSize;

    namespace optimalcontrol {

        /**
         * @warning This class is still in active development, and so API interface can change between iDynTree versions.
         * \ingroup iDynTreeExperimental
         */

        class OptimizationInstance {

        public:

            OptimizationInstance();

            virtual ~OptimizationInstance();

            virtual bool prepare();

            virtual bool getInfo(unsigned int& numberOfVariables, unsigned int& numberOfConstraints,
                                 unsigned int& numberOfNonZerosConstraintsJacobian, unsigned int& numberOfNonZerosHessian);

            virtual bool getBoundsInfo(VectorDynSize& variablesLowerBounds, VectorDynSize& variableUpperBounds,
                                       VectorDynSize& constraintsLowerBounds, VectorDynSize& constraintsUpperBounds,
                                       double infinity = 1e19);

           /* virtual bool getStartingPoint(VectorDynSize& variablesGuess,
                                          VectorDynSize& lowerBoundsMultipliersGuess,
                                          VectorDynSize& upperBoundsMultipliersGuess,
                                          VectorDynSize& constraintsMultiplierGuess); */ //This method should be part of the solver interface

            virtual bool evaluateCostFunction(const VectorDynSize& variables, double& costValue);

            virtual bool evaluateCostGradient(const VectorDynSize& variables, VectorDynSize& gradient);

            virtual bool evaluateConstraints(const VectorDynSize& variables, VectorDynSize& constraints);

            virtual bool evaluateConstraintsJacobian(const VectorDynSize& variables, SparseMatrix<RowMajor>& jacobian);

            virtual bool evaluateHessian(const VectorDynSize& variables, double costMultiplier,
                                         const VectorDynSize& constraintsMultipliers, SparseMatrix<RowMajor>& hessian);

            //virtual bool getSolution(); this method should be part of the solver interface (?)
        };
    }
}

#endif // IDYNTREE_OPTIMALCONTROL_OPTIMIZATIONINSTANCE_H
