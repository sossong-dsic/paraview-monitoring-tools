#ifndef __AvroEventReader_h
#define __AvroEventReader_h

#include <boost/filesystem.hpp>

#include <avro/Encoder.hh>
#include <avro/Decoder.hh>
#include <avro/ValidSchema.hh>
#include <avro/Compiler.hh>
#include <avro/DataFile.hh>

#include <icomm/marshal/AvroAdapter.h>
#include <das-avro-types/EventPacket.hh>

template < typename AvroMessage >
class AvroEventReader
{
    typedef typename boost::shared_ptr< AvroMessage > AvroMessageSharedPtr;

public:
    AvroEventReader(std::string base_data_dir, bool verbose = false)
    : base_data_dir_(base_data_dir)
    , extension_(".avro")
    , df_eof(false)
    , verbose_(verbose)
    {}

    AvroEventReader(bool verbose = true)
    : base_data_dir_("/home/sossong/opt/dsc/")
    , extension_(".avro")
    , df_eof(false)
    , verbose_(verbose)
    {}

    ~AvroEventReader() {}

    /**
     * Read the next message from the file if one exists.
     */
    AvroMessageSharedPtr Read()
    {
      AvroMessageSharedPtr avro_data;

      bool read_success = false;
      while (!read_success)
      {
        if (df_reader_ == NULL)
        {
          if (avro_file_iter_ != avro_file_list_.end())
          {
            do
            {
              avro_data = Read((*avro_file_iter_));
              avro_file_iter_++;
            } while ((avro_file_iter_ != avro_file_list_.end()) && (avro_data == NULL));
          }
          else
          {
            // Reached the end of the list, inform the caller
            return NULL;
          }
          if (!df_eof)
            read_success = true;  // Mark that we got a good packet from a new file
        }

        if (!read_success)  // Skip this if we already got a good packet from a new file
        {
          AvroMessage avro_d;
          if (!(df_eof = !df_reader_->read(avro_d)))
          {
            // Still got a good packet from this file
            avro_data = NewAvroMessage(avro_d);
            read_success = true;
          }
          else
          {
            // No more packets in this file
            df_reader_.reset();  // Reset the reader so that we read from a new file
          }
        }
      }

      return (avro_data);
    }

    AvroMessageSharedPtr Read(const std::string& data_file)
    {

      df_reader_.reset(new avro::DataFileReader< AvroMessage >(data_file.c_str()));
      // Need to return true or false to tell if the file read was good. Restructure this to return data through reference

      AvroMessage avro_data;
      df_eof = !df_reader_->read(avro_data);

      AvroMessageSharedPtr avro_packet = NewAvroMessage(avro_data);

      return (avro_packet);
    }

    /** Uses Base Data Directory to get file paths */
    void GetFilePaths()
    {
        boost::filesystem::path p(base_data_dir_);
        if (boost::filesystem::is_directory(p))
        {
            for (boost::filesystem::recursive_directory_iterator end, dir(p); dir !=end; ++dir)
            {
                if (!boost::filesystem::is_directory(dir->path()))
                {
                    avro_file_list_.push_back(dir->path().string());
                }
            }
            PrintFileList();
        }
        else
        {
            std::cout << "base_data_dir is not a directory: " << base_data_dir_ << std::endl;
        }
        avro_file_iter_ = avro_file_list_.begin();
    }
    void AppendFilePath(const std::string& full_path_filename)
    {
      avro_file_list_.push_back(full_path_filename);
      avro_file_iter_ = avro_file_list_.begin();  // really should do this only once after they're all added, but screw it!
    }
    

      AvroMessageSharedPtr NewAvroMessage(AvroMessage message)
      {
        AvroMessageSharedPtr avro_message(new AvroMessage(message));
        return (avro_message);
      }

      void PrintFileList()
      {
        printf("%s\n", __PRETTY_FUNCTION__);
        std::vector< std::string >::const_iterator file_iter = avro_file_list_.begin();
        for (; file_iter != avro_file_list_.end(); ++file_iter)
        {
          printf("  %s\n", (*file_iter).c_str());
        }
        printf("\n");
      }

private:
      std::string base_data_dir_;
      std::string extension_;
      std::string type_;
      std::string time_fmt_;
      bool df_eof;
      boost::shared_ptr< avro::DataFileReader< AvroMessage > > df_reader_;
      std::vector< std::string > avro_file_list_;
      std::vector< std::string >::const_iterator avro_file_iter_;
      bool verbose_;
};
#endif /* __AvroEventReader_h */
