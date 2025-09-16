
#ifndef _CTRL_H_
#define _CTRL_H_

/* CODE */
namespace robotun
{
    namespace ctrl
    {
        class pid_ctrl
        {
        public:
            pid_ctrl(double Kp_, double Ki_, double Kd_, double target_, double last_state_ = 0);
            ~pid_ctrl();

            double get_ctrl(double value, double dt);
            void set_target(double target_);
        private:
            double Kp, Ki, Kd;
            double target;
            double last_state;
            double integral_eror;
        protected:
        };

        class lqr_ctrl
        {
        public:
            lqr_ctrl();
            ~lqr_ctrl();

            double lqr();
            double dlqr();
        private:
        protected:
        };

    }   /* end namespace ctrl */
}   /* end namespace robotun*/

#endif





