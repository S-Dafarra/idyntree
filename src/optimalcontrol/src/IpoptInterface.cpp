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

#define HAVE_STDDEF_H
#define HAVE_CSTDDEF
#include <IpTNLP.hpp>
#undef HAVE_STDDEF_H
#undef HAVE_CSTDDEF //workaroud for missing libraries
#include <IpIpoptApplication.hpp>

#include <Eigen/Dense>
#include <iDynTree/Core/EigenHelpers.h>

#include <iDynTree/Core/VectorDynSize.h>
#include "iDynTree/Optimizers/IpoptInterface.h"
#include "iDynTree/Core/Utils.h"

#include <cassert>

namespace iDynTree {
    namespace optimization {

        class NLPImplementation : public Ipopt::TNLP {
            // IPOPT methods redefinition
            unsigned int m_numberOfConstraints;
            VectorDynSize m_constraintsLowerBounds, m_constraintsUpperBounds, m_variablesLowerBounds, m_variablesUpperBounds;
            std::vector<size_t> m_constraintsJacNNZRows, m_constraintsJacNNZCols, m_hessianNNZRows, m_hessianNNZCols;
        public:

            std::shared_ptr<OptimizationProblem> problem;
            double minusInfinity, plusInfinity; //TODO. Set these before solving

            NLPImplementation()
            :minusInfinity(-1e19)
            ,plusInfinity(1e19)
            {}

            virtual ~NLPImplementation() override {}

            virtual bool get_nlp_info(Ipopt::Index& n, Ipopt::Index& m, Ipopt::Index& nnz_jac_g,
                              Ipopt::Index& nnz_h_lag, IndexStyleEnum& index_style) override{
                if (!problem) {
                    reportError("NLPImplementation", "get_nlp_info", "OptimizationProblem not set.");
                    return false;
                }

                n = static_cast<Ipopt::Index>(problem->numberOfVariables());

                if (!(problem->getConstraintsInfo(m_numberOfConstraints, m_constraintsLowerBounds, m_constraintsUpperBounds))){
                    reportError("NLPImplementation", "get_nlp_info", "Error while retrieving constraints info.");
                    return false;
                }
                m = static_cast<Ipopt::Index>(m_numberOfConstraints);

                if (!(problem->getConstraintsJacobianInfo(m_constraintsJacNNZRows, m_constraintsJacNNZCols))){
                    reportError("NLPImplementation", "get_nlp_info", "Error while retrieving constraints jacobian info.");
                    return false;
                }

                nnz_jac_g = static_cast<Ipopt::Index>(m_constraintsJacNNZRows.size());

                if (!(problem->getHessianInfo(m_hessianNNZRows, m_hessianNNZCols))){
                    reportError("NLPImplementation", "get_nlp_info", "Error while retrieving hessian info.");
                    return false;
                }

                nnz_h_lag = static_cast<Ipopt::Index>(m_hessianNNZRows.size());

                index_style = C_STYLE;
                return true;
            }


            virtual bool get_bounds_info(Ipopt::Index n, Ipopt::Number* x_l, Ipopt::Number* x_u,
                                 Ipopt::Index m, Ipopt::Number* g_l, Ipopt::Number* g_u) override {
                if (!problem) {
                    reportError("NLPImplementation", "get_bounds_info", "OptimizationProblem not set.");
                    return false;
                }

                bool variablesLowerBounded = false, variablesUpperBounded = false;
                if (problem->getVariablesLowerBound(m_variablesLowerBounds)){
                    variablesLowerBounded = true;
                }
                if (problem->getVariablesUpperBound(m_variablesUpperBounds)){
                    variablesUpperBounded = true;
                }

                Eigen::Map<Eigen::VectorXd> variablesLB(x_l, n);
                Eigen::Map<Eigen::VectorXd> variablesUB(x_u, n);
                Eigen::Map<Eigen::VectorXd> variablesLBInput = toEigen(m_variablesLowerBounds);
                Eigen::Map<Eigen::VectorXd> variablesUBInput = toEigen(m_variablesUpperBounds);

                if (variablesLowerBounded)
                    variablesLB = variablesLBInput;
                else variablesLB.setConstant(minusInfinity);

                if (variablesUpperBounded)
                    variablesUB = variablesUBInput;
                else variablesUB.setConstant(plusInfinity);

                //The constraints bounds have been already saved in the get_nlp_info method.

                Eigen::Map<Eigen::VectorXd> constraintsLowerBounds(g_l, m);
                Eigen::Map<Eigen::VectorXd> constraintsUpperBounds(g_u, m);

                constraintsLowerBounds = toEigen(m_constraintsLowerBounds);
                constraintsUpperBounds = toEigen(m_constraintsUpperBounds);

                return true;
            }

            bool get_starting_point(Ipopt::Index n, bool init_x, Ipopt::Number* x,
                                    bool init_z, Ipopt::Number* z_L, Ipopt::Number* z_U,
                                    Ipopt::Index m, bool init_lambda, Ipopt::Number* lambda)
            {}

            bool eval_f(Ipopt::Index n, const Ipopt::Number* x,
                        bool new_x, Ipopt::Number& obj_value)
            {}

            bool eval_grad_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                             Ipopt::Number* grad_f)
            {}

            bool eval_g(Ipopt::Index n, const Ipopt::Number* x,
                        bool new_x, Ipopt::Index m, Ipopt::Number* g)
            {}

            bool eval_jac_g(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                            Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index* iRow,
                            Ipopt::Index *jCol, Ipopt::Number* values)
            {}

            bool eval_h(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                        Ipopt::Number obj_factor, Ipopt::Index m, const Ipopt::Number* lambda,
                        bool new_lambda, Ipopt::Index nele_hess, Ipopt::Index* iRow,
                        Ipopt::Index* jCol, Ipopt::Number* values)
            {}

            void finalize_solution(Ipopt::SolverReturn status, Ipopt::Index n,
                                   const Ipopt::Number* x, const Ipopt::Number* z_L,
                                   const Ipopt::Number* z_U, Ipopt::Index m, const Ipopt::Number* g,
                                   const Ipopt::Number* lambda, Ipopt::Number obj_value,
                                   const Ipopt::IpoptData* ip_data,
                                   Ipopt::IpoptCalculatedQuantities* ip_cq)
            {}
        };

        class IpoptInterface::IpoptInterfaceImplementation {
        public:
            Ipopt::SmartPtr<NLPImplementation> nlpPointer;
            Ipopt::SmartPtr<Ipopt::IpoptApplication> loader;
            IpoptInterfaceImplementation()
            :nlpPointer(new NLPImplementation())
            ,loader(IpoptApplicationFactory())
            {
            }
        };

        IpoptInterface::IpoptInterface()
        :m_pimpl(new IpoptInterfaceImplementation())
        {
            assert(m_pimpl);
        }

        IpoptInterface::~IpoptInterface()
        {
            if (m_pimpl){
                delete m_pimpl;
                m_pimpl = nullptr;
            }
        }

        bool IpoptInterface::setProblem(std::shared_ptr<OptimizationProblem> problem)
        {
            if (!problem){
                reportError("IpoptInterface", "setProblem", "Empty problem pointer.");
                return false;
            }
            m_problem = problem;
            m_pimpl->nlpPointer->problem = problem;
            return true;
        }

        double IpoptInterface::minusInfinity()
        {
            Ipopt::Number output;
            bool ok = m_pimpl->loader->Options()->GetNumericValue("nlp_lower_bound_inf", output, "");
            if (!ok){
                reportWarning("IpoptInterface", "minusInfinity", "Error while reading the nlp_lower_bound_inf value from Ipopt. Returning the default.");
                return Optimizer::minusInfinity();
            }
            return static_cast<double>(output);
        }

        double IpoptInterface::plusInfinity()
        {
            Ipopt::Number output;
            bool ok = m_pimpl->loader->Options()->GetNumericValue("nlp_upper_bound_inf", output, "");
            if (!ok){
                reportWarning("IpoptInterface", "minusInfinity", "Error while reading the nlp_upper_bound_inf value from Ipopt. Returning the default.");
                return Optimizer::plusInfinity();
            }
            return static_cast<double>(output);
        }


    }
}
