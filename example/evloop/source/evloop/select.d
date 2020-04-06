module evloop.select;

version (select):

import core.sys.posix.sys.select;

import evloop.base;

class SelectLoop : EvLoop
{
    private bool isRun;

    FDData[int] fds;

    this()
    {
        isRun = true;
    }

    override
    {
        bool step()
        {
            fd_set read_set, write_set, err_set;

            FD_ZERO(&read_set);
            FD_ZERO(&write_set);
            FD_ZERO(&err_set);

            int nfds = 0;

            foreach (fd, data; fds)
            {
                //mlogf("SET %d %s", fd, data);
                if (data.repeat == 0) continue;

                if (data.read)
                {
                    FD_SET(fd, &read_set);
                    if (fd > nfds) nfds = fd;
                }
                if (data.write)
                {
                    FD_SET(fd, &write_set);
                    if (fd > nfds) nfds = fd;
                }
                FD_SET(fd, &err_set);
            }

            nfds++;

            //mlog("SELECT");
            check!select(nfds, &read_set, &write_set, &err_set, null);
            //mlog("   SELECT FIN");

            foreach (fd, data; fds)
            {
                const isread = FD_ISSET(fd, &read_set);
                const iswrite = FD_ISSET(fd, &write_set);
                const iserr = FD_ISSET(fd, &err_set);
                const setted = isread || iswrite || iserr;

                if (!setted) continue;

                //mlogf("CHECK FD %d %s read(%s) write(%s) err(%s)",
                //    fd, data, isread, iswrite, iserr);

                if (data.repeat == 0) continue;

                if (data.repeat > 0) data.repeat--;

                if (iserr)
                {
                    (*data.cb)(EvType.ERR);
                    continue;
                }

                (*data.cb)( (isread ? EvType.READ : 0) |
                           (iswrite ? EvType.WRITE : 0) );
            }

            return isRun;
        }

        void stop() { isRun = false; }

        void run() { while (step()) {} }

        void addFD(int fd, FDData data) { fds[fd] = data; }
        void modFD(int fd, FDData data) { fds[fd] = data; }
        void delFD(int fd) { fds.remove(fd); }

        Waker makeNewWaker(Worker w)
        { return new BaseWaker(w); }

        Timer makeNewTimer(void delegate(Timer) fnc)
        {
            import evloop.linuxtimer;
            return new LinuxTimer(fnc);
        }
    }
}

static this() { evl = new SelectLoop; }
