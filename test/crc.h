/*

struct CrcModule {
    BaseModule &_base_;
    int _assig_ = -1;

    /// @b COMPLETE
    inline bool polynomial_degree(const int &degree) noexcept {
      auto poly_r = std::distance(std::begin(_base_.crcpoly_ref),
        std::find(std::begin(_base_.crcpoly_ref), 
          std::end(_base_.crcpoly_ref), degree));

      if (poly_r < std::size(_base_.crcpoly_ref)) {
        _base_._reg_->CRCCTRL.reg &= ~DMAC_CRCCTRL_CRCPOLY_Msk;
        _base_._reg_->CRCCTRL.reg |= (poly_r << DMAC_CRCCTRL_CRCPOLY_Pos);
        return true;
      }
      return false;
    } 
    /// @b COMPLETE
    inline int polynomial_degree() const noexcept {
      return _base_.crcpoly_ref[_base_._reg_->CRCCTRL.reg 
        & DMAC_CRCCTRL_CRCPOLY_Msk];
    }

    /// @b COMPLETE
    inline bool enabled(const bool &value) noexcept {
      if (value && _assig_ == -1) return false;
      _base_._reg_->CRCCTRL.reg &= ~DMAC_CRCCTRL_CRCSRC_Msk;
      _base_._reg_->CRCCTRL.reg |= ((value ? _assig_ 
        : DMAC_CRCCTRL_CRCSRC_DISABLE_Val) << DMAC_CRCCTRL_CRCSRC_Pos);
      return true;
    }
    /// @b COMPLETE
    inline bool enabled() const noexcept {
      return !(_base_._reg_->CRCCTRL.reg & DMAC_CRCCTRL_CRCSRC_DISABLE);
    }

    /// @b COMPLETE
    inline bool input_source(const int &index) noexcept {
      if (!is_active()) {
        _assig_ = (index >= 0 && index < DMAC_CH_NUM)
          ? index + _base_.crc_src_shift : DMAC_CRCCTRL_CRCSRC_IO_Val;
        _base_._reg_->CRCCTRL.reg &= ~DMAC_CRCCTRL_CRCSRC_Msk;
        _base_._reg_->CRCCTRL.reg |= (_assig_ << DMAC_CRCCTRL_CRCSRC_Pos);
        return true;
      }
      return false;
    }
    /// @b COMPLETE
    inline int input_source() const noexcept {
      auto reg_v = (_base_._reg_->CRCCTRL.reg & DMAC_CRCCTRL_CRCSRC_Msk);
      if (reg_v == DMAC_CRCCTRL_CRCSRC_DISABLE_Val) {
        return -2;
      } else if (reg_v == DMAC_CRCCTRL_CRCSRC_IO_Val) {
        return -1;
      }
      return reg_v + _base_.crc_src_shift;
    }

    inline bool is_active() const noexcept {
      auto reg_v = (_base_._reg_->CRCCTRL.reg & DMAC_CRCCTRL_CRCSRC_Msk);
      return reg_v == DMAC_CRCCTRL_CRCSRC_DISABLE_Val ? false
        : (_base_._reg_->CRCSTATUS.reg & DMAC_CRCSTATUS_CRCBUSY);
    }

    inline decltype(std::declval<DMAC_CRCCHKSUM_Type>().reg) output() 
      const noexcept {
      if (_base_._reg_->CRCCTRL.reg & DMAC_CRCCTRL_CRCPOLY_CRC16) {
        return _base_._reg_->CRCCHKSUM.reg;
      } else {
        
      }
    }

    inline bool reset_checksum() {
      if ((_base_._reg_->CRCCTRL.reg & DMAC_CRCCTRL_CRCSRC_Msk)
        == DMAC_CRCCTRL_CRCSRC_DISABLE_Val) {
        _base_._reg_->CRCCHKSUM.reg = DMAC_CRCCHKSUM_RESETVALUE;
      }
    }

  };

*/