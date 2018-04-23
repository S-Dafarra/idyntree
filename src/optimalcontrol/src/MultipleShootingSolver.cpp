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

#include <Eigen/Dense>
#include <iDynTree/Core/EigenHelpers.h>

#include <vector>
#include <cassert>
#include <algorithm>
#include <cmath>
#include <string>
#include <sstream>

namespace iDynTree {
    namespace optimalcontrol
    {

        enum class MeshPointType{
            Control,
            State
        };

        class MeshPointOrigin{
            std::string m_name;
            std::string m_description;
            int m_priority;
        public:
            MeshPointOrigin(){}
            MeshPointOrigin(const std::string& name, int priority, const std::string& description) : m_name(name), m_description(description), m_priority(priority){}
            static MeshPointOrigin FirstPoint() {return MeshPointOrigin("FirstPoint", 9, "The first point");}
            static MeshPointOrigin LastPoint() {return MeshPointOrigin("LastPoint", 8, "The last point");}
            static MeshPointOrigin Control() {return MeshPointOrigin("Control", 7, "A control mesh");}
            static MeshPointOrigin CompensateLongPeriod() {return MeshPointOrigin("CompensateLongPeriod", 6, "A mesh filling a long period");}
            static MeshPointOrigin UserControl() {return MeshPointOrigin("UserControl", 5, "An user defined control mesh");}
            static MeshPointOrigin UserState() {return MeshPointOrigin("UserState", 4, "An user defined state mesh");}
            static MeshPointOrigin Instant() {return MeshPointOrigin("Instant", 3, "A mesh corresponding to a single instant cost/constraint");}
            static MeshPointOrigin FillVariables() {return MeshPointOrigin("FillVariables", 2, "A mesh added to have a constant number of varialbes");}
            static MeshPointOrigin TimeRange() {return MeshPointOrigin("TimeRange", 1, "A mesh corresponding to the begin/end of a cost/constraint");}
            static MeshPointOrigin Ignored() {return MeshPointOrigin("Ignored", 0, "An ignored mesh");}

            int priority() const {return m_priority;}
            std::string name() const {return m_name;}
            std::string description() const {return m_description;}

            bool operator<(const MeshPointOrigin &rhs) const {return m_priority < rhs.priority();}
            bool operator==(const MeshPointOrigin &rhs) const {return m_priority == rhs.priority();}
            bool operator!=(const MeshPointOrigin &rhs) const {return m_priority != rhs.priority();}
        };

        typedef  struct{
            double time;
            MeshPointType type;
            MeshPointOrigin origin;
            size_t controlOffset, stateOffset;
            //std::vector<size_t> integratorAuxiliariesOffsets;
        } MeshPoint;

        class MultipleShootingTranscription::MultipleShootingTranscriptionPimpl{
        public:
            std::shared_ptr<OptimalControlProblem> ocproblem;
            std::shared_ptr<Integrator> integrator;
            size_t totalMeshes, controlMeshes;
            unsigned int nonZerosJacobian, nonZerosHessian;
            bool prepared;
            std::vector<double> userStateMeshes, userControlMeshes;
            std::vector<MeshPoint> meshPoints;
            std::vector<MeshPoint>::iterator meshPointsEnd;
            double minStepSize, maxStepSize, controlPeriod;
            size_t numberOfVariables;

            void resetMeshPoints(){
                meshPointsEnd = meshPoints.begin();
            }

            void addMeshPoint(MeshPoint& newMeshPoint){
                if (meshPointsEnd == meshPoints.end()){
                    meshPoints.push_back(newMeshPoint);
                    meshPointsEnd = meshPoints.end();
                } else {
                    *meshPointsEnd = newMeshPoint;
                    meshPointsEnd++;
                }
            }

            std::vector<MeshPoint>::iterator findNextMeshPoint(std::vector<MeshPoint>::iterator& start){
                std::vector<MeshPoint>::iterator nextMesh = start;
                MeshPointOrigin ignored = MeshPointOrigin::Ignored();
                do {
                    nextMesh++;
                    assert(nextMesh != meshPoints.end());
                } while (nextMesh->origin == ignored); //find next valid mesh

                return nextMesh;
            }

            void setIgnoredMesh(std::vector<MeshPoint>::iterator& toBeIgnored){
                assert(toBeIgnored != meshPoints.end());
                toBeIgnored->time = ocproblem->finalTime() + maxStepSize;
                toBeIgnored->origin = MeshPointOrigin::Ignored();
            }

            void cleanLeftoverMeshes(){
                for (auto toBeIgnored = meshPointsEnd; toBeIgnored != meshPoints.end(); toBeIgnored++){
                    setIgnoredMesh(toBeIgnored);
                }
            }

            void priorityWarning(std::vector<MeshPoint>::iterator& meshToBeRemoved, MeshPointOrigin& noWarningLevel){
                if (noWarningLevel < meshToBeRemoved->origin){ //check if priority was high
                    std::ostringstream errorMsg;
                    errorMsg << meshToBeRemoved->origin.description() << " had to be removed due to limits on the minimum step size. "
                             << "Its time was " << meshToBeRemoved->time;
                    reportWarning("MultipleShootingSolver", "setMeshPoints", errorMsg.str().c_str());
                }
            }

            MultipleShootingTranscriptionPimpl()
            : ocproblem(nullptr)
            ,integrator(nullptr)
            ,totalMeshes(0)
            ,controlMeshes(0)
            ,nonZerosJacobian(0)
            ,nonZerosHessian(0)
            ,prepared(false)
            ,meshPointsEnd(meshPoints.end())
            ,minStepSize(0.001)
            ,maxStepSize(0.01)
            ,controlPeriod(0.01)
            ,numberOfVariables(0)
            {}

            MultipleShootingTranscriptionPimpl(const std::shared_ptr<OptimalControlProblem> problem,
                                            const std::shared_ptr<Integrator> integrationMethod)
            :ocproblem(problem)
            ,integrator(integrationMethod)
            ,totalMeshes(0)
            ,controlMeshes(0)
            ,nonZerosJacobian(0)
            ,nonZerosHessian(0)
            ,prepared(false)
            ,meshPointsEnd(meshPoints.end())
            ,minStepSize(0.001)
            ,maxStepSize(0.01)
            ,controlPeriod(0.01)
            ,numberOfVariables(0)
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

        size_t MultipleShootingTranscription::setControlMeshPoints()
        {
            double endTime = m_pimpl->ocproblem->finalTime(), initTime = m_pimpl->ocproblem->initialTime();
            size_t controlMeshes = 0;
            double time = initTime;
            MeshPoint newMeshPoint;

            newMeshPoint.type = MeshPointType::Control;

            while (time <= endTime){
                if (time == initTime)
                    newMeshPoint.origin = MeshPointOrigin::FirstPoint();
                else if (time == endTime)
                    newMeshPoint.origin = MeshPointOrigin::LastPoint();
                else
                    newMeshPoint.origin = MeshPointOrigin::Control();
                newMeshPoint.time = time;

                m_pimpl->addMeshPoint(newMeshPoint);
                time += m_pimpl->controlPeriod;
                controlMeshes++;
            }

            newMeshPoint.origin = MeshPointOrigin::LastPoint();
            newMeshPoint.type = MeshPointType::State;
            newMeshPoint.time = endTime;

            if (m_pimpl->meshPoints[controlMeshes - 1].time < endTime){
                if ((m_pimpl->meshPoints[controlMeshes - 1].time + m_pimpl->minStepSize) > endTime){ //if last control mesh point is too close to the end, remove it and place a state mesh point at the end
                    m_pimpl->meshPoints[controlMeshes - 1] = newMeshPoint;
                    controlMeshes--;
                } else {
                    m_pimpl->addMeshPoint(newMeshPoint);  //otherwise add a mesh point at the end;
                }
            }
            return controlMeshes;
        }


        bool MultipleShootingTranscription::setMeshPoints()
        {
            double endTime = m_pimpl->ocproblem->finalTime(), initTime = m_pimpl->ocproblem->initialTime();
            m_pimpl->resetMeshPoints();

            if (m_pimpl->controlPeriod < 2*(m_pimpl->minStepSize)){ //no other mesh point could be inserted otherwise
                size_t controlMeshes = setControlMeshPoints();
                size_t totalMeshes = static_cast<size_t>(m_pimpl->meshPointsEnd - m_pimpl->meshPoints.begin());

                if (m_pimpl->prepared){
                    if ((m_pimpl->totalMeshes != totalMeshes) || (m_pimpl->controlMeshes != controlMeshes)){
                        reportWarning("MultipleShootingSolver", "setMeshPoints", "Unable to keep number of variables constant.");
                    }
                }

                m_pimpl->controlMeshes = controlMeshes;
                m_pimpl->totalMeshes = totalMeshes;
                return true;
            }

            setControlMeshPoints();

            MeshPoint newMeshPoint;
            //Adding user defined control mesh points
            newMeshPoint.origin = MeshPointOrigin::UserControl();
            newMeshPoint.type = MeshPointType::Control;
            for (size_t i = 0; i < m_pimpl->userControlMeshes.size(); ++i){
               if ((m_pimpl->userControlMeshes[i] > initTime) && (m_pimpl->userControlMeshes[i] < endTime)){
                    newMeshPoint.time = m_pimpl->userControlMeshes[i];
                    m_pimpl->addMeshPoint(newMeshPoint);
               } else {
                   std::ostringstream errorMsg;
                   errorMsg << "Ignored user defined control mesh point at time " << m_pimpl->userControlMeshes[i] << "Out of time range." << std::endl;
                   reportWarning("MultipleShootingSolver", "setMeshPoints", errorMsg.str().c_str());
               }
            }

            //Adding user defined state mesh points
            newMeshPoint.origin = MeshPointOrigin::UserState();
            newMeshPoint.type = MeshPointType::State;
            for (size_t i = 0; i < m_pimpl->userStateMeshes.size(); ++i){
                if ((m_pimpl->userStateMeshes[i] > initTime) && (m_pimpl->userStateMeshes[i] < endTime)){
                    newMeshPoint.time = m_pimpl->userStateMeshes[i];
                    m_pimpl->addMeshPoint(newMeshPoint);
                } else {
                    std::ostringstream errorMsg;
                    errorMsg << "Ignored user defined state mesh point at time " << m_pimpl->userStateMeshes[i] << "Out of time range." << std::endl;
                    reportWarning("MultipleShootingSolver", "setMeshPoints", errorMsg.str().c_str());
                }
            }

            std::vector<TimeRange> &constraintsTRs = m_pimpl->ocproblem->getConstraintsTimeRanges();
            std::vector<TimeRange> &costsTRs = m_pimpl->ocproblem->getCostsTimeRanges();

            //adding mesh points for the constraint time ranges
            newMeshPoint.type = MeshPointType::State;
            for (size_t i = 0; i < constraintsTRs.size(); ++i){
                if (constraintsTRs[i].isInstant() && !(constraintsTRs[i].isInRange(initTime))){
                    newMeshPoint.origin = MeshPointOrigin::Instant();
                    newMeshPoint.time = constraintsTRs[i].initTime();
                    m_pimpl->addMeshPoint(newMeshPoint);
                }
                else {
                    newMeshPoint.origin = MeshPointOrigin::TimeRange();
                    if (constraintsTRs[i].initTime() > initTime){
                        newMeshPoint.time = constraintsTRs[i].initTime();
                        m_pimpl->addMeshPoint(newMeshPoint);
                    }
                    if (constraintsTRs[i].endTime() < endTime){
                        newMeshPoint.time = constraintsTRs[i].endTime();
                        m_pimpl->addMeshPoint(newMeshPoint);
                    }
                }
            }


            //adding mesh points for the costs time ranges
            for (size_t i = 0; i < costsTRs.size(); ++i){
                if (costsTRs[i].isInstant() && !(costsTRs[i].isInRange(initTime))){
                    newMeshPoint.origin = MeshPointOrigin::Instant();
                    newMeshPoint.time = costsTRs[i].initTime();
                    m_pimpl->addMeshPoint(newMeshPoint);
                }
                else {
                    newMeshPoint.origin = MeshPointOrigin::TimeRange();
                    if (costsTRs[i].initTime() > initTime){
                        newMeshPoint.time = costsTRs[i].initTime();
                        m_pimpl->addMeshPoint(newMeshPoint);
                    }
                    if (costsTRs[i].endTime() < endTime){
                        newMeshPoint.time = costsTRs[i].endTime();
                        m_pimpl->addMeshPoint(newMeshPoint);
                    }
                }
            }

            m_pimpl->cleanLeftoverMeshes(); //setto ignored the leftover meshes, i.e. those that were set in a previous call

            double tMin = m_pimpl->minStepSize;
            std::sort(m_pimpl->meshPoints.begin(), m_pimpl->meshPointsEnd,
                      [tMin](const MeshPoint&a, const MeshPoint&b) {
                            if (std::abs(b.time - a.time) < tMin)
                                return a.origin.priority() < b.origin.priority();
                            else return a.time < b.time;}); //reorder the vector. Using this ordering, by scrolling the vector, in case of two narrow points, I will get the more important first

            //Now I need to prune the vector

            std::vector<MeshPoint>::iterator mesh = m_pimpl->meshPoints.begin();
            std::vector<MeshPoint>::iterator nextMesh = mesh;
            MeshPointOrigin last = MeshPointOrigin::LastPoint();
            MeshPointOrigin ignored = MeshPointOrigin::Ignored();
            MeshPointOrigin timeRangeOrigin = MeshPointOrigin::TimeRange();

            newMeshPoint.origin = MeshPointOrigin::CompensateLongPeriod();
            newMeshPoint.type = MeshPointType::State;

            double timeDistance;
            while (mesh->origin != last){

                nextMesh = m_pimpl->findNextMeshPoint(mesh); //find next valid mesh
                timeDistance = std::abs(nextMesh->time - mesh->time); //for the way I have ordered the vector, it can be negative

                if (timeDistance > m_pimpl->maxStepSize){ //two consecutive points are too distant
                    unsigned int additionalPoints = static_cast<unsigned int>(std::ceil(timeDistance/(m_pimpl->maxStepSize))) - 1;
                    double dtAdd = timeDistance / (additionalPoints + 1); //additionalPoints + 1 is the number of segments.
                    //since tmin < tmax/2, dtAdd > tmin. Infact, the worst case is when timeDistance is nearly equal to tmax.
                    for (unsigned int i = 0; i < additionalPoints; ++i) {
                        newMeshPoint.time = mesh->time + (i + 1)*dtAdd;
                        m_pimpl->addMeshPoint(newMeshPoint);
                    }
                    mesh = nextMesh;

                } else if (timeDistance < m_pimpl->minStepSize){ //too consecutive points are too close

                    if (nextMesh->origin < mesh->origin){ //check who has the higher priority
                        m_pimpl->priorityWarning(nextMesh, timeRangeOrigin);
                        m_pimpl->setIgnoredMesh(nextMesh);
                    } else {
                        m_pimpl->priorityWarning(mesh, timeRangeOrigin);
                        m_pimpl->setIgnoredMesh(mesh);
                        do {
                            mesh--;
                        } while (mesh->origin == ignored);
                        nextMesh = mesh;
                    }
                } else {
                    mesh = nextMesh;
                }
            }

            std::sort(m_pimpl->meshPoints.begin(), m_pimpl->meshPointsEnd,
                      [](const MeshPoint&a, const MeshPoint&b) {return a.time < b.time;}); //reorder the vector

            size_t totalMeshes = 0, controlMeshes = 0;
            mesh = m_pimpl->meshPoints.begin();

            while (mesh->origin != last){ //count meshes
                if (mesh->type == MeshPointType::Control){
                    controlMeshes++;
                }
                totalMeshes++;
                mesh++;
            }
            totalMeshes++; //the last mesh
            m_pimpl->meshPointsEnd = m_pimpl->meshPoints.begin() + static_cast<long>(totalMeshes);
            assert((m_pimpl->meshPointsEnd - 1)->origin == last);

            if (m_pimpl->prepared){
                if (m_pimpl->controlMeshes == controlMeshes){

                    if (m_pimpl->totalMeshes != totalMeshes){

                        if (m_pimpl->totalMeshes < totalMeshes){ //here I need to remove meshes

                            size_t toBeRemoved = totalMeshes - m_pimpl->totalMeshes;
                            mesh = m_pimpl->meshPoints.begin();
                            nextMesh = mesh;
                            std::vector<MeshPoint>::iterator nextNextMesh = mesh;
                            while ((mesh->origin != last) && (toBeRemoved > 0)){
                                nextMesh = m_pimpl->findNextMeshPoint(mesh); //find next valid mesh

                                if (nextMesh->origin != last) {
                                    nextNextMesh = m_pimpl->findNextMeshPoint(nextMesh);
                                    timeDistance = std::abs(nextNextMesh->time - mesh->time);

                                    if ((timeDistance < m_pimpl->maxStepSize) && !(timeRangeOrigin < nextMesh->origin)){ //I can remove nextMesh
                                        m_pimpl->setIgnoredMesh(nextMesh);
                                        toBeRemoved--;
                                        totalMeshes--;
                                        mesh = nextNextMesh;
                                    } else {
                                        mesh = nextMesh;
                                    }
                                } else {
                                    mesh = nextMesh;
                                }
                            }
                            if (toBeRemoved > 0){
                                reportWarning("MultipleShootingSolver", "setMeshPoints",
                                              "Unable to keep a constant number of variables. Unable to remove enough mesh points.");
                            }

                        } else if (m_pimpl->totalMeshes > totalMeshes){ //here I need to add meshes
                            unsigned int toBeAdded = static_cast<unsigned int>(m_pimpl->totalMeshes - totalMeshes);
                            mesh = m_pimpl->meshPoints.begin();
                            nextMesh = mesh;
                            newMeshPoint.origin = MeshPointOrigin::FillVariables();
                            newMeshPoint.type = MeshPointType::State;
                            while ((mesh->origin != last) && (toBeAdded > 0)){
                                nextMesh = m_pimpl->findNextMeshPoint(mesh); //find next valid mesh
                                timeDistance = std::abs(nextMesh->time - mesh->time);

                                if (timeDistance > (2*m_pimpl->minStepSize)){
                                    unsigned int possibleMeshes = static_cast<unsigned int>(std::ceil(timeDistance/(m_pimpl->minStepSize))) - 1;
                                    unsigned int meshToAddHere = std::min(toBeAdded, possibleMeshes);
                                    double dT = timeDistance/(meshToAddHere + 1);
                                    for(unsigned int m = 1; m <= meshToAddHere; m++){
                                        newMeshPoint.time = mesh->time + m*dT;
                                        m_pimpl->addMeshPoint(newMeshPoint);
                                        totalMeshes++;
                                    }
                                }
                            }

                            if (toBeAdded > 0){
                                reportWarning("MultipleShootingSolver", "setMeshPoints",
                                              "Unable to keep a constant number of variables. Unable to add enough mesh points.");
                            }

                        }
                        std::sort(m_pimpl->meshPoints.begin(), m_pimpl->meshPointsEnd,
                                  [](const MeshPoint&a, const MeshPoint&b) {return a.time < b.time;}); //reorder the vector
                        m_pimpl->meshPointsEnd = m_pimpl->meshPoints.begin() + static_cast<long>(totalMeshes);
                        assert((m_pimpl->meshPointsEnd - 1)->origin == last);
                    }
                } else {
                    reportWarning("MultipleShootingSolver", "setMeshPoints",
                                  "Unable to keep a constant number of variables. The control meshes are different");
                }
            }

            m_pimpl->totalMeshes = totalMeshes;
            m_pimpl->controlMeshes = controlMeshes;

            return true;
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
                reportError("MultipleShootingSolver", "setOptimalControlProblem", "The OptimalControlProblem for this instance has already been set.");
                return false;
            }

            m_pimpl->ocproblem = problem;
            return true;
        }

        bool MultipleShootingTranscription::setIntegrator(const std::shared_ptr<Integrator> integrationMethod)
        {
            if (m_pimpl->integrator){
                reportError("MultipleShootingSolver", "setIntegrator", "The integration method for this instance has already been set.");
                return false;
            }

            m_pimpl->integrator = integrationMethod;
            return true;
        }

        bool MultipleShootingTranscription::setStepSizeBounds(const double minStepSize, const double maxStepsize)
        {
            if (minStepSize <= 0){
                reportError("MultipleShootingSolver", "setStepSizeBounds","The minimum step size is expected to be positive.");
                return false;
            }

            if (maxStepsize <= (2 * maxStepsize)){
                reportError("MultipleShootingSolver", "setStepSizeBounds","The maximum step size is expected to be greater than twice the minimum."); //imagine to have a distance between two mesh points slightly greater than the max. It would be impossible to add a mesh point in the middle
                return false;
            }

            m_pimpl->minStepSize = minStepSize;
            m_pimpl->maxStepSize = maxStepsize;

            return true;
        }

        bool MultipleShootingTranscription::setControlPeriod(double period)
        {
            if (period <= 0){
                reportError("MultipleShootingSolver", "prepare", "The control period is supposed to be positive.");
                return false;
            }
            m_pimpl->controlPeriod = period;
            return true;
        }

        bool MultipleShootingTranscription::setAdditionalStateMeshPoints(const std::vector<double> &stateMeshes)
        {
            m_pimpl->userStateMeshes = stateMeshes;
            return true;
        }

        bool MultipleShootingTranscription::setAdditionalControlMeshPoints(const std::vector<double> &controlMeshes)
        {
            m_pimpl->userControlMeshes = controlMeshes;
            return true;
        }

        bool MultipleShootingTranscription::prepare()
        {
            //TODO: Once it is clear from where this method will be called, change the error messages.

            if ((m_pimpl->ocproblem->finalTime() - m_pimpl->ocproblem->initialTime()) < m_pimpl->minStepSize){
                reportError("MultipleShootingTranscription", "prepare",
                            "The time horizon defined in the OptimalControlProblem is smaller than the minimum step size.");
                return false;
            }

            if (m_pimpl->controlPeriod < m_pimpl->minStepSize){
                reportError("MultipleShootingTranscription", "prepare", "The control period cannot be lower than the minimum step size.");
                return false;
            }

            if ((m_pimpl->controlPeriod >= m_pimpl->maxStepSize) && (m_pimpl->controlPeriod <= (2 * m_pimpl->minStepSize))){
                reportError("MultipleShootingTranscription", "prepare",
                            "Cannot add control mesh points when the controller period is in the range [dTMax, 2*dTmin]."); // the first mesh point is a control mesh. The second mesh than would be too far from the first but not far enough to put a state mesh point in the middle.
                return false;
            }

            if (!m_pimpl->prepared){
                if (m_pimpl->ocproblem->dynamicalSystem().expired()){
                    if (m_pimpl->integrator->dynamicalSystem().expired()){
                        reportError("MultipleShootingTranscription", "prepare",
                                    "No dynamical system set, neither to the OptimalControlProblem nor to the Integrator object.");
                        return false;
                    }
                    if (!(m_pimpl->ocproblem->setDynamicalSystemConstraint(m_pimpl->integrator->dynamicalSystem().lock()))){
                        reportError("MultipleShootingTranscription", "prepare",
                                    "Error while setting the dynamicalSystem to the OptimalControlProblem using the one pointer provided by the Integrator object.");
                        return false;
                    }
                } else {
                    if (!(m_pimpl->integrator->dynamicalSystem().expired()) &&
                            (m_pimpl->integrator->dynamicalSystem().lock() != m_pimpl->ocproblem->dynamicalSystem().lock())){
                        reportError("MultipleShootingTranscription", "prepare",
                                    "The selected OptimalControlProblem and the Integrator point to two different dynamical systems.");
                        return false;
                    }
                    if (!(m_pimpl->integrator->setDynamicalSystem(m_pimpl->ocproblem->dynamicalSystem().lock()))){
                        reportError("MultipleShootingTranscription", "prepare",
                                    "Error while setting the dynamicalSystem to the Integrator using the one pointer provided by the OptimalControlProblem object.");
                        return false;
                    }
                }
            }

            if (!(m_pimpl->integrator->setMaximumStepSize(m_pimpl->maxStepSize))){
                reportError("MultipleShootingTranscription", "prepare","Error while setting the maximum step size to the integrator.");
                return false;
            }

            if (!setMeshPoints()){
                return false;
            }
            size_t nx = m_pimpl->integrator->dynamicalSystem().lock()->stateSpaceSize();
            size_t nu = m_pimpl->integrator->dynamicalSystem().lock()->controlSpaceSize();
            //TODO: I should consider also the possibility to have auxiliary variables in the integrator
            m_pimpl->numberOfVariables = (m_pimpl->totalMeshes - 1) * nx + m_pimpl->controlMeshes * nu; //the -1 removes the initial state from the set of optimization varibales
            std::vector<MeshPoint>::iterator mesh = m_pimpl->meshPoints.begin();
            size_t index = 0, previousControlIndex = 0;
            MeshPointOrigin first = MeshPointOrigin::FirstPoint();
            while (mesh != m_pimpl->meshPointsEnd){
                if (mesh->origin == first){
                    previousControlIndex = index;
                    mesh->controlOffset = index;
                    index += nu;
                } else if (mesh->type == MeshPointType::Control) {
                    previousControlIndex = index;
                    mesh->controlOffset = index;
                    index += nu;
                    mesh->stateOffset = index;
                    index += nx;
                } else if (mesh->type == MeshPointType::State) {
                    mesh->controlOffset = previousControlIndex;
                    mesh->stateOffset = index;
                    index += nx;
                }
                mesh++;
            }
            assert(index == m_pimpl->numberOfVariables);

            //here I still need to count the constraints and the number of nonzeros.

            m_pimpl->prepared = true;
            return true;
        }

        void MultipleShootingTranscription::reset()
        {
            m_pimpl->prepared = false;
            m_pimpl->totalMeshes = 0;
            m_pimpl->controlMeshes = 0;
        }

        bool MultipleShootingTranscription::getInfo(unsigned int &numberOfVariables, unsigned int &numberOfConstraints,
                                                 unsigned int &numberOfNonZerosConstraintsJacobian, unsigned int &numberOfNonZerosHessian)
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
        {
            assert(m_pimpl);
            m_transcription.reset(new MultipleShootingTranscription());
            assert(m_transcription);
            m_transcription->setOptimalControlProblem(ocProblem);
        }

        bool MultipleShootingSolver::setStepSizeBounds(double minStepSize, double maxStepsize)
        {
            return m_transcription->setStepSizeBounds(minStepSize, maxStepsize);
        }

        bool MultipleShootingSolver::setIntegrator(const std::shared_ptr<Integrator> integrationMethod)
        {
            return m_transcription->setIntegrator(integrationMethod);
        }

        bool MultipleShootingSolver::setControlPeriod(double period)
        {
            return m_transcription->setControlPeriod(period);
        }

        bool MultipleShootingSolver::setAdditionalStateMeshPoints(const std::vector<double> &stateMeshes)
        {
            return m_transcription->setAdditionalStateMeshPoints(stateMeshes);
        }

        bool MultipleShootingSolver::setAdditionalControlMeshPoints(const std::vector<double> &controlMeshes)
        {
            return m_transcription->setAdditionalControlMeshPoints(controlMeshes);
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
