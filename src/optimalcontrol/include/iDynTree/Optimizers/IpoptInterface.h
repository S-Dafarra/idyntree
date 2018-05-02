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

#ifndef IDYNTREE_OPTIMALCONTROL_IPOPTINTERFACE_H
#define IDYNTREE_OPTIMALCONTROL_IPOPTINTERFACE_H

#include <memory>
#include <string>
#include <iDynTree/Optimizer.h>

namespace iDynTree {

    class VectorDynSize;

    namespace optimization {

        /**
         * @warning This class is still in active development, and so API interface can change between iDynTree versions.
         * \ingroup iDynTreeExperimental
         */

        class IpoptInterface : public Optimizer {

            class IpoptInterfaceImplementation;
            IpoptInterfaceImplementation *m_pimpl;

        public:

            IpoptInterface();

            IpoptInterface(const IpoptInterface &other) = delete;

            virtual ~IpoptInterface();

            virtual bool setProblem(std::shared_ptr<OptimizationProblem> problem) final;

            virtual bool setInitialGuess(VectorDynSize &initialGuess) final;

            virtual bool solve() final;

            virtual bool getPrimalVariables(VectorDynSize &primalVariables) final;

            virtual bool getDualVariables(VectorDynSize &constraintsMultipliers,
                                          VectorDynSize &lowerBoundsMultipliers,
                                          VectorDynSize &upperBoundsMultipliers) final;

            virtual bool getOptimalCost(double &optimalCost) final;

            virtual bool getOptimalConstraintsValues(VectorDynSize &constraintsValues) final;

            virtual double minusInfinity() final;

            virtual double plusInfinity() final;

            bool setIpoptOption(std::string &tag, std::string &value); //TO BE IMPLEMENTED

            bool setIpoptOption(std::string &tag, double &value);

            bool setIpoptOption(std::string &tag, int &value);
        };

    }

}

#endif // IDYNTREE_OPTIMALCONTROL_IPOPTINTERFACE_H
