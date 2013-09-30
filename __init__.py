#Copyright 2013 Aaron Curtis

#Licensed under the Apache License, Version 2.0 (the "License");
#you may not use this file except in compliance with the License.
#You may obtain a copy of the License at

    #http://www.apache.org/licenses/LICENSE-2.0

#Unless required by applicable law or agreed to in writing, software
#distributed under the License is distributed on an "AS IS" BASIS,
#WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#See the License for the specific language governing permissions and
#limitations under the License.

import pandas, datetime, utils, string, pdb, numpy
import pylab, sqlite3
from pymavlink import mavutil

#datatypes defined in ardupilot/libraries/DataFlash/DataFlash.h
fmt_units={
  'b'   : 'int8_t',
  'B'   : 'uint8_t',
  'h'   : 'int16_t',
  'H'   : 'uint16_t',
  'i'   : 'int32_t',
  'I'   : 'uint32_t',
  'f'   : 'float',
  'n'   : 'char[4]',
  'N'   : 'char[16]',
  'Z'   : 'char[64]',
  'c'   : 'int16_t * 100',
  'C'   : 'uint16_t * 100',
  'e'   : 'int32_t * 100',
  'E'   : 'uint32_t * 100',
  'L'   : 'int32_t latitude/longitude',
  'M'   : 'uint8_t flight mode'
}
fmt_dtypes={
  'b'   : int,
  'B'   : int,
  'h'   : int,
  'H'   : int,
  'i'   : int,
  'I'   : int,
  'f'   : float,
  'n'   : str,
  'N'   : str,
  'Z'   : str,
  'c'   : float,
  'C'   : float,
  'e'   : float,
  'E'   : float,
  'L'   : float,
  'M'   : str
}

class flight():
    logdata=None
    flight_name=None
    def __init__(self, dflog_path, messaging=None, **time_kwargs):
        self.read_dflog(dflog_path)
        self.set_dtype_from_fmt()
        
        #if no starttime info, use the filename date
        dt=utils.logpath2dt(dflog_path)
        self.flight_name=utils.slugify(dflog_path.split('/')[-1])
        self.set_times_from_ublox(year=dt.year, month=dt.month, day=dt.day)

    def read_dflog(self, dflog_path, start_time=None, max_cols=20):
        """
        Reads in a APM dataflash .log file, returning it as a pandas DataFrame.
        File must be in the self-describing format (no legacy support yet).
    
        Given the path to a log file produced by the ArduPilotMega Mission Planner,
        stores a dict of :class:`pandas.DataFrame`, in self.logdata, with one
        DataFrame for each message type.    
        """
        dflog_data=pandas.read_csv(dflog_path,header=None,names=string.lowercase[:max_cols],skipinitialspace=True)
        #Group rows by command type and make a dictionary where command types are keys, values are dataframes
        dflog_by_msg={k:v for k, v in dflog_data.groupby('a',sort=False)}
        for fmt_row in dflog_by_msg['FMT'].iterrows():
            try:
                log_msg_type=fmt_row[1]['d'].strip()
                #Drop null columns
                dflog_by_msg[log_msg_type]=dflog_by_msg[log_msg_type].dropna(axis=1,how='all')
                #Select data for one command 
                dflog_msg=dflog_by_msg[log_msg_type]
                #Remove message name
                del dflog_msg['a']
                #Set column labels based on FMT
                if log_msg_type == 'FMT':
                    #TODO set column names on FMT table. Opens a can of worms.
                    continue
                msg_col_labels=fmt_row[1]['f':].dropna()
                dflog_msg.columns=msg_col_labels[:len(dflog_msg.columns)]
                dflog_by_msg[log_msg_type]=dflog_msg
            except KeyError:
                print "No %s information" % log_msg_type
                continue
        self.logdata=dflog_by_msg
    
    
    def save_logdict_as_hdf5(self):
        raise NotImplementedError
    
    def set_dtype_from_fmt(self):
        """
        Uses data in the FMT tables to write correct column headers to an APM
        dataflash .log file in the dict of :class:`pandas.DataFrame` format.
        """
        fmt_dtype_cols=self.logdata['FMT'].ix[:,['d','e']]
        for row in fmt_dtype_cols.iterrows():
            cmd_name=row[1]['d']
            cmd_dtypes_str=row[1]['e']
            try:
                cmd_dataframe=self.logdata[cmd_name]
            except KeyError:
                continue
            for col, dtype_char in zip(cmd_dataframe.iteritems(), cmd_dtypes_str):
                cmd_param_name, col_data = col
                self.logdata[cmd_name][cmd_param_name]=col[1].astype(fmt_dtypes[dtype_char])
    
    def set_times_from_ublox(self, week_of_year=None, year=None, month=None, day=None, epoch=None):
        """
        Replaces line numbers in the log data with datetimes.
        """
        #Find time lag between lines
        first_last_df=self.logdata['GPS']['Time'].iloc[[0,-1]]
        f,l=first_last_df
        f_logline,l_logline=first_last_df.index
        ms_per_logline=(l-f)/float((l_logline-f_logline))
    
        #Add the epoch for this particular GPS format
        if not epoch:
            if week_of_year and year:
                epoch=datetime.datetime(year=year,month=1,day=1)+datetime.timedelta(week_of_year)
            if year and month:
                epoch=datetime.datetime(year=year,month=month,day=day)
        #increment the epoch by the first gps timestamp TODO: check for problem when no lock
        epoch+=datetime.timedelta(milliseconds=int(f))
        linenum2time=lambda x : epoch + datetime.timedelta(milliseconds=x * ms_per_logline)

        for cmd_table in self.logdata.values():
            cmd_table.rename_axis(linenum2time,inplace=True)
    
    def plot(self, suppress=('APM 2','FMT','D32','PM','MODE','PARM','ArduCopter','Free RAM','CMD'), **kwargs):
        """
        Plots the data in a single column of subplots.
        """
        pylab.ion()
        fig=pylab.figure(figsize=(14,50))
        #remove the data that isn't worth plotting
        pltdata={k:v for k,v in self.logdata.iteritems() if not k.startswith(suppress)}
        ncols=1
        nrows=len(pltdata)/ncols
        for idx, msg_name in enumerate(pltdata):
            # idx + 1 because enumerate starts at 0 but pylab indexes starting at 1
            pdb.set_trace()
            subpl=fig.add_subplot(nrows,ncols,idx)
            self.logdata[msg_name].plot(ax=subpl, title=msg_name, **kwargs)
    
    def to_afterflight_msgtbl(self,msg_type,flight_id=None):
        msgtbl=self.logdata[msg_type]
        num_rows=msgtbl.shape[0]
        msgtbl=pandas.DataFrame(zip(
                    [msg_type,]*num_rows,
                    [flight_id or self.flight_name,]*num_rows,
                    msgtbl.index))
        msgtbl.columns=['msgType','flight_id','timestamp']
        msgtbl['timestamp']=msgtbl.timestamp.apply(lambda x: x.isoformat().replace('T',' '))
        return msgtbl

    def to_afterflight_datatbl(self,msg_type):
        datatbl=self.logdata[msg_type].stack()
        datatbl=pandas.DataFrame(zip(
                    datatbl.index.get_level_values(0),
                    datatbl.index.get_level_values(1),
                    datatbl.values))
        datatbl.columns=['timestamp','msgField','value']
        datatbl['timestamp']=datatbl.timestamp.apply(lambda x: x.isoformat().replace('T',' '))
        return datatbl
    
    def to_afterflight_flighttbl(self,flight_id=None):
        flighttbl=pandas.DataFrame([self.flight_name,])
        flighttbl.columns=['slug']
        return flighttbl
        
    def to_afterflight_sql(self, dbconn=None, close_when_done=True, db_name='flyingrhi', flight_id=None):
        if not dbconn:
            dbconn=sqlite3.connect('flyrhi.db')
        try:
            flighttbl=self.to_afterflight_flighttbl(flight_id)
            flighttbl.to_sql('logbrowse_flight',dbconn,if_exists='append')
        except dbconn.IntegrityError:
            print "Flight %s already exists in the database. Not creating." % self.flight_name 
        for msg_type in self.logdata:
                msgtbl=self.to_afterflight_msgtbl(msg_type,flight_id)
                msgtbl.to_sql('logbrowse_mavmessage',dbconn,if_exists='append')
                datatbl=self.to_afterflight_datatbl(msg_type)
                datatbl.to_sql('logbrowse_mavdatum',dbconn,if_exists='append')
                print "Processed " + msg_type
        if close_when_done:
            dbconn.close()