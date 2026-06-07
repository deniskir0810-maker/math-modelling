/**
* @file include/three_bodies_problem_solver.hpp
* @author Denis Kirilenko
* 
* Ðåøàëêà äëÿ çàäà÷è òðåõ òåë
*/

#ifndef INCLUDE_THREE_BODY_PROBLEM_SOLVER_HPP_
#define INCLUDE_THREE_BODY_PROBLEM_SOLVER_HPP_

#include "abstract_solver.hpp"
#include "matpoint.hpp"

namespace mm {

  /**
   * @brief Ðåøàëêà çàäà÷è òð¸õ òåë ìåòîäîì Ðóíãå-Êóòòû 4-ãî ïîðÿäêà.
   * @tparam T Òèï äàííûõ (float/double).
   */
template<typename T>
class ThreeBodyProblemSolver : public AbstractSolver<T> {
private:
  mat_point<T> bodies[3];  //!< Òðè ìàòåðèàëüíûå òî÷êè

  /**
   * @brief Êîïèðóåò òåêóùåå ñîñòîÿíèå â ìàññèâ.
   * @param dest Ìàññèâ èç òð¸õ òî÷åê äëÿ çàïîëíåíèÿ.
   */
  void GetState(mat_point<T>(&dest)[3]) const;

    /**
     * @brief Óñòàíàâëèâàåò ñîñòîÿíèå èç ìàññèâà.
     * @param src Ìàññèâ ñ íîâûì ñîñòîÿíèåì.
     */
  void SetState(const mat_point<T>(&src)[3]);

    /**
     * @brief Âû÷èñëÿåò ïðîèçâîäíûå (dx/dt = v, dv/dt = a) äëÿ çàäàííîãî ñîñòîÿíèÿ.
     * @param state Òåêóùåå ñîñòîÿíèå (ïîëîæåíèÿ, ñêîðîñòè, ìàññû).
     * @param[out] dstate Ïðîèçâîäíûå: dstate[i].point = ñêîðîñòü, dstate[i].velocity = óñêîðåíèå.
     */
  void ComputeDerivatives(const mat_point<T>(&state)[3], mat_point<T>(&dstate)[3]) const;

public:
     /**
      * @brief Êîíñòðóêòîð.
      * @param body0 Ïåðâîå òåëî.
      * @param body1 Âòîðîå òåëî.
      * @param body2 Òðåòüå òåëî.
      * @param tau Øàã ïî âðåìåíè.
      * @param finishTime Êîíå÷íîå âðåìÿ.
      * @param exportPeriod Ïåðèîä ýêñïîðòà.
      */
  ThreeBodyProblemSolver(const mat_point<T>& body0, const mat_point<T>& body1, const mat_point<T>& body2,
    T tau, T finishTime, T exportPeriod);

    bool MakeStep() override;
    void ExportData(nlohmann::json* output) override;
  };

} // namespace mm

#include "three_bodies_problem_solver_impl.hpp"

#endif // INCLUDE_THREE_BODY_PROBLEM_SOLVER_HPP_
