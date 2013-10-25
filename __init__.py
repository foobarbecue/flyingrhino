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

import pandas, datetime, utils, string, numpy, pdb
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
        #calculate the beginning of the week from logfile name for ublox timestamps
        week_epoch=utils.logpath2dt(dflog_path)
        self.read_dflog(dflog_path, epoch=week_epoch)
        self.set_dtype_from_fmt()
        self.flight_name=utils.slugify(dflog_path.split('/')[-1])
        self.messaging=messaging
        
    def read_dflog(self, dflog_path, start_time=None, max_cols=20, epoch=None):
        """
        Reads in a APM dataflash .log file, returning it as a pandas DataFrame.
        File must be in the self-describing format (no legacy support yet).
    
        Given the path to a log file produced by the ArduPilotMega Mission Planner,
        stores a dict of :class:`pandas.DataFrame`, in self.logdata, with one
        DataFrame for each message type.    
        """
        dflog_data=pandas.read_csv(dflog_path,header=None,names=string.lowercase[:max_cols],skipinitialspace=True)
        self.num_lines=dflog_data.shape[0]
        #interpolate gps timestamps
        gpslines=dflog_data['a']=='GPS'
        dflog_data['timestamp']=dflog_data['c'][gpslines]
        dflog_data['orig_linenum']=dflog_data.index
        dflog_data.timestamp=dflog_data.timestamp.interpolate()
        #remove non-null duplicates
        dflog_data=dflog_data[~(~dflog_data.timestamp.isnull() & dflog_data.timestamp.duplicated())]
        #turn timestamp values into datetimes if we have a starting week
        def gpststamp2dt(timestamp):
            if not numpy.isnan(timestamp):
                return epoch + datetime.timedelta(milliseconds=timestamp)
        if epoch:
            dflog_data['timestamp']=dflog_data.timestamp.apply(gpststamp2dt)
        
        #Group rows by command type and make a dictionary where command types are keys, values are dataframes
        dflog_by_msg={k:v for k, v in dflog_data.groupby('a',sort=False)}
        #modify format table so that all messages receive a timestamp column
        dflog_by_msg['FMT']['timestamp']=['timestamp']*len(dflog_by_msg['FMT'])
        dflog_by_msg['FMT']['orig_linenum']=['orig_linenum']*len(dflog_by_msg['FMT'])
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
    
    def find_armed_segments(self):
        gpst=self.logdata['GPS']
        #Create a boolean column that's True if this row exceeds the gap threshold
        gpst['gaps'] = gpst.Time.diff() > 400
        #Add the beginning and the end of the log
        gpst['gaps'].iloc[[0,-1]] = True
        #Use the boolean column to select those rows
        gap_edges=gpst[gpst['gaps']]
        gaps=[]
        gap_edge_lns=gap_edges.index
        for ind in range(len(gap_edge_lns)-1):
            gaps.append((gap_edge_lns[ind], gap_edge_lns[ind+1]))
        #This was to avoid glitches by not putting timestamps on things less than 1 sec. But, we need timestamps on everything
        #b/c it's a PK for MavMessage.
        #gaps_cleaned=[gap for gap in gaps if gap[1]-gap[0] > 1000]
        return gaps

    def set_times_from_ublox_segment(self, start_linenum, end_linenum, week_of_year=None, year=None, month=None, day=None, epoch=None):
            #Find time lag between lines
        first_last_df=self.logdata['GPS']['Time'].loc[[start_linenum,end_linenum]]
        f,l=first_last_df
        f_logline,l_logline=first_last_df.index
        ms_per_logline=(l-f)/float((l_logline-f_logline))

        #increment the epoch by the first gps timestamp TODO: check for problem when no lock
        epoch+=datetime.timedelta(milliseconds=int(f))
        
        def linenum2time(linenum):
            try:
                if start_linenum < linenum < end_linenum:
                    return epoch + datetime.timedelta(milliseconds=linenum * ms_per_logline)
                else:
                    return linenum
            except TypeError:
                return linenum

        for cmd_table in self.logdata.values():
            cmd_table.rename_axis(linenum2time,inplace=True)    
    
    def set_times_from_ublox(self, week_of_year=None, year=None, month=None, day=None, epoch=None, assume_continuous=True):
        """
        Replaces line numbers in the log data with datetimes.
        """
        #Add the epoch for this particular GPS format
        if not epoch:
            if week_of_year and year:
                epoch=datetime.datetime(year=year,month=1,day=1)+datetime.timedelta(week_of_year)
            if year and month:
                epoch=datetime.datetime(year=year,month=month,day=day)
        segments=self.find_armed_segments()
        #Store the original line number index in a new column before we overwrite them with dates.
        for cmd_table_name in self.logdata.keys():
            self.logdata[cmd_table_name]['orig_linenum']=self.logdata[cmd_table_name].index
        #Replace line numbers with linearly interpolated timestamps for each segment
        for segment in segments:
            self.set_times_from_ublox_segment(segment[0], segment[1], epoch=epoch)
        #Delete lines that didn't get a timestamp (before or after first / last GPS line)
        for cmd_table_name in self.logdata.keys():
            cmd_table=self.logdata[cmd_table_name]
            self.logdata[cmd_table_name]=cmd_table[cmd_table.orig_linenum!=cmd_table.index]
    
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
            subpl=fig.add_subplot(nrows,ncols,idx)
            self.logdata[msg_name].plot(ax=subpl, title=msg_name, **kwargs)
        
    def to_afterflight_msgtbl(self,msg_type,flight_id=None):   
        msgtbl=self.logdata[msg_type]
        msgtbl=msgtbl[msgtbl.timestamp!=msgtbl.orig_linenum]
        msgtbl=msgtbl[msgtbl.timestamp.notnull()]
        num_rows=msgtbl.shape[0]
        msgtbl=pandas.DataFrame(zip(
                    [msg_type,]*num_rows,
                    [flight_id or self.flight_name,]*num_rows,
                    msgtbl.timestamp))
        msgtbl.columns=['msgType','flight_id','timestamp']
        msgtbl['timestamp']=msgtbl.timestamp.apply(lambda x: x.isoformat().replace('T',' '))
        return msgtbl

    def to_afterflight_datatbl(self,msg_type):
        datatbl=self.logdata[msg_type]
        datatbl=datatbl[datatbl.timestamp!=datatbl.orig_linenum]
        datatbl=datatbl[datatbl.timestamp.notnull()]       
        datatbl.index=datatbl.timestamp
        datatbl=datatbl.stack()
        datatbl=pandas.DataFrame(zip(
                    datatbl.index.get_level_values(0),
                    datatbl.index.get_level_values(1),
                    datatbl.values))
        datatbl.columns=['timestamp','msgField','value']
        datatbl['timestamp']=datatbl.timestamp.apply(lambda x: x.isoformat().replace('T',' '))
        return datatbl
    
    def to_afterflight_paramtbl(self,flight_id=None):
        """
        Special case for making a table of MavDatum objects for flight parameters
        """
        paramtbl=self.logdata['PARM']      
        #paramtbl.index=paramtbl.timestamp
        paramtbl.Name=paramtbl.Name.apply(lambda x: 'PARM-'+x)
        num_rows=paramtbl.shape[0]
        paramtbl=pandas.DataFrame(zip([flight_id or self.flight_name,]*num_rows,
                                  paramtbl.Name,
                                  paramtbl.Value))
        paramtbl.columns=['flight_id','name', 'value']
        return paramtbl
    
    def to_afterflight_flighttbl(self,flight_id=None):
        flighttbl=pandas.DataFrame([self.flight_name,])
        flighttbl.columns=['slug']
        return flighttbl
        
    def to_afterflight_sql(self, dbconn=None, close_when_done=True, db_name='flyingrhi', flight_id=None):
        #todo: msg_type is the same as message_table_name. Should unify variable names
        if not dbconn:
            dbconn=sqlite3.connect('flyrhi.db')
        try:
            flighttbl=self.to_afterflight_flighttbl(flight_id)
            flighttbl.to_sql('logbrowse_flight',dbconn,if_exists='append')
        except dbconn.IntegrityError:
            print "Flight %s already exists in the database. Not creating." % self.flight_name 
        for ind, msg_type in enumerate(self.logdata):
            msgtbl=self.logdata[msg_type]
            if msg_type not in ['CMD','FMT','PARM'] and len(self.logdata[msg_type]) > 2:
                msgtbl=self.to_afterflight_msgtbl(msg_type,flight_id)
                msgtbl.to_sql('logbrowse_mavmessage',dbconn,if_exists='append')
                datatbl=self.to_afterflight_datatbl(msg_type)
                datatbl.to_sql('logbrowse_mavdatum',dbconn,if_exists='append')            
                print "Processed " + msg_type
                if self.messaging:
                    self.messaging("Processed " + msg_type, length=len(self.logdata), uploaded=ind)
            else:
                print "Ignored empty frame" + msg_type
            paramtbl=self.to_afterflight_paramtbl(msg_type)
            paramtbl.to_sql('logbrowse_param',dbconn,if_exists='append')
        if close_when_done:
            dbconn.close()