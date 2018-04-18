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

#include "iDynTree/OptimizationInstance.h"
#include "iDynTree/Core/Utils.h"

namespace iDynTree {

    namespace optimalcontrol {

    OptimizationInstance::OptimizationInstance()
    {
    }

    OptimizationInstance::~OptimizationInstance()
    {
    }

    bool OptimizationInstance::prepare()
    {
        reportError("OptimizationInstance", "prepare", "Method not implemented.");
        return false;
    }

    void OptimizationInstance::reset()
    {

    }

    bool OptimizationInstance::getInfo(unsigned int &numberOfVariables, unsigned int &numberOfConstraints,
                                           unsigned int &numberOfNonZerosConstraintsJacobian, unsigned int &numberOfNonZerosHessian)
        {
            reportError("OptimizationInstance", "getInfo", "Method not implemented.");
            return false;
        }

        bool OptimizationInstance::getBoundsInfo(VectorDynSize &variablesLowerBounds, VectorDynSize &variableUpperBounds,
                                                 VectorDynSize &constraintsLowerBounds, VectorDynSize &constraintsUpperBounds,
                                                 double infinity /* = 1e19*/)
        {
            reportError("OptimizationInstance", "getBoundsInfo", "Method not implemented.");
            return false;
        }

        bool OptimizationInstance::evaluateCostFunction(const VectorDynSize &variables, double &costValue)
        {
            reportError("OptimizationInstance", "evaluateCostFunction", "Method not implemented.");
            return false;
        }

        bool OptimizationInstance::evaluateCostGradient(const VectorDynSize &variables, VectorDynSize &gradient)
        {
            reportError("OptimizationInstance", "evaluateCostGradient", "Method not implemented.");
            return false;
        }

        bool OptimizationInstance::evaluateConstraints(const VectorDynSize &variables, VectorDynSize &constraints)
        {
            reportError("OptimizationInstance", "evaluateConstraints", "Method not implemented.");
            return false;
        }

        bool OptimizationInstance::evaluateConstraintsJacobian(const VectorDynSize &variables, SparseMatrix<RowMajor> &jacobian)
        {
            reportError("OptimizationInstance", "evaluateConstraintsJacobian", "Method not implemented.");
            return false;
        }

        bool OptimizationInstance::evaluateHessian(const VectorDynSize &variables, double costMultiplier,
                                                   const VectorDynSize &constraintsMultipliers,  SparseMatrix<RowMajor> &hessian)
        {
            reportError("OptimizationInstance", "evaluateHessian", "Method not implemented.");
            return false;
        }


    }
}
